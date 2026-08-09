/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2023 by Tatu Peltola OH2EAT
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
 *   Copyright (C) 2026 by Adrian Musceac YO8RZZ
 *   Copyright (C) 2026 by Shawn Chain BG5HHP
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if defined(USE_SOAPY)

#include "Globals.h"
#include "Config.h"

#include "SDRSoapy.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cassert>

#if !defined(M_PI)
#define M_PI    3.141592654
#endif

const size_t RX_CHANNEL = 0;
const size_t TX_CHANNEL = 0;

const size_t LATENCY_BLOCKS = 3;
const size_t LIBRE_TX_QUEUE_BLOCKS = 64;

const int32_t FM_DEVIATION = 550000;

// To allow for fine tuning of the deviation levels
const q15_t LEVEL_50PC_INVERTED = -128 * 128;
const q15_t LEVEL_40PC_INVERTED = -102 * 128;
const q15_t LEVEL_30PC_INVERTED = -77  * 128;
const q15_t LEVEL_100PC         =  255 * 128;


CSDRSoapy::CSDRSoapy() :
m_trace(false),
m_started(false),
m_rxBuffer(RX_RINGBUFFER_SIZE, "IO RX Buffer"),
m_txBuffer(TX_RINGBUFFER_SIZE, "IO TX Buffer"),
m_power(0.0F),
m_txFreq(0U),
m_rxFreq(0U),
m_pocsagFreq(0U),
m_rxGain(50.0F),
m_txGain(30.0F),
m_soapyTXFreq(0.0),
m_soapyRXFreq(0.0),
m_soapyPocsagFreq(0.0),
m_soapyInit(false),
m_timestamped(false),
m_latencyNs(0LL),
m_phase(0U),
m_prevRXIQSample(0.0F, 0.0F),
m_delayedTXBuffer(nullptr),
m_buffer(),
m_fdudc(nullptr),
m_soapyDeviceType("sx"),
m_soapyDeviceURI(),
m_device(nullptr),
m_rxStream(nullptr),
m_txStream(nullptr),
m_useTXWorker(false),
m_txWorker(),
m_txQueueMutex(),
m_txQueueCondition(),
m_txQueue(),
m_txWorkerRunning(false),
m_txWorkerFailed(false),
m_txQueueDrops(0U),
m_pocsag(false)
{
}

CSDRSoapy::~CSDRSoapy()
{
}

bool CSDRSoapy::start(bool trace)
{
  if (m_started)
    return true;

  m_trace = trace;

  m_started = true;

  LogMessage("Soapy SDR started");

  return true;
}

void CSDRSoapy::stop()
{
  stopTXWorker();

  delete m_fdudc;
  m_fdudc = nullptr;

  delete m_delayedTXBuffer;
  m_delayedTXBuffer = nullptr;

  if (m_device != nullptr) {
    assert(m_rxStream != nullptr);
    assert(m_txStream != nullptr);

    if (m_soapyInit) {
      m_device->deactivateStream(m_rxStream, 0, 0);
      m_device->deactivateStream(m_txStream, 0, 0);
    }

    m_device->closeStream(m_rxStream);
    m_device->closeStream(m_txStream);

    SoapySDR::Device::unmake(m_device);
  }

  m_rxStream = nullptr;
  m_txStream = nullptr;
  m_device   = nullptr;

  m_soapyInit = false;
}

bool CSDRSoapy::writeTXBlock(const std::vector<std::complex<float>>& samples, int flags, long long timeNs)
{
  size_t samplesWritten = 0U;

  while (samplesWritten < samples.size()) {
    void* buffs[1] = {
      const_cast<std::complex<float>*>(samples.data() + samplesWritten)
    };

    int chunkFlags = flags;
    long long chunkTimeNs = timeNs;

    // A timestamp applies to the first sample in the complete block only.
    if (samplesWritten > 0U) {
      chunkFlags &= ~SOAPY_SDR_HAS_TIME;
      chunkTimeNs = 0LL;
    }

    const int ret = m_device->writeStream(
      m_txStream, buffs, samples.size() - samplesWritten, chunkFlags, chunkTimeNs);

    if (ret <= 0) {
      LogError("TX stream error after %zu/%zu samples: %d (%s)",
               samplesWritten, samples.size(), ret, SoapySDR_errToStr(ret));
      return false;
    }

    samplesWritten += static_cast<size_t>(ret);
  }

  return true;
}

void CSDRSoapy::startTXWorker()
{
  if (!m_useTXWorker)
    return;

  stopTXWorker();

  {
    std::lock_guard<std::mutex> lock(m_txQueueMutex);
    m_txQueue.clear();
  }

  m_txQueueDrops = 0U;
  m_txWorkerFailed.store(false);
  m_txWorkerRunning.store(true);
  m_txWorker = std::thread(&CSDRSoapy::txWorkerLoop, this);
}

void CSDRSoapy::stopTXWorker()
{
  m_txWorkerRunning.store(false);
  m_txQueueCondition.notify_all();

  if (m_txWorker.joinable())
    m_txWorker.join();

  std::lock_guard<std::mutex> lock(m_txQueueMutex);
  m_txQueue.clear();
}

void CSDRSoapy::enqueueTXBlock(const std::vector<std::complex<float>>& samples, int flags, long long timeNs)
{
  if (!m_useTXWorker || !m_txWorkerRunning.load() || m_txWorkerFailed.load())
    return;

  {
    std::lock_guard<std::mutex> lock(m_txQueueMutex);

    if (m_txQueue.size() >= LIBRE_TX_QUEUE_BLOCKS) {
      m_txQueue.pop_front();
      m_txQueueDrops++;

      if (m_txQueueDrops == 1U || (m_txQueueDrops % 100U) == 0U)
        LogError("LibreSDR TX queue overflow: dropped oldest block, total drops=%llu",
                 static_cast<unsigned long long>(m_txQueueDrops));
    }

    m_txQueue.push_back({samples, flags, timeNs});
  }

  m_txQueueCondition.notify_one();
}

void CSDRSoapy::txWorkerLoop()
{
  using Clock = std::chrono::steady_clock;

  bool diagnosticActive = false;
  bool havePreviousWriteEnd = false;
  Clock::time_point previousWriteEnd;
  uint64_t blocks = 0U;
  uint64_t calls = 0U;
  uint64_t shortWrites = 0U;
  uint64_t under2ms = 0U;
  uint64_t from2to3ms = 0U;
  uint64_t from3to5ms = 0U;
  uint64_t from5to10ms = 0U;
  uint64_t over10ms = 0U;
  uint64_t startingDrops = 0U;
  uint64_t latestDrops = 0U;
  size_t maxQueueDepth = 0U;
  double maxWriteMs = 0.0;
  double maxSubmitGapMs = 0.0;

  const auto finishDiagnostic = [&]() {
    if (!diagnosticActive)
      return;

    LogMessage(
      "LibreSDR TX worker diagnostic: blocks=%llu calls=%llu short_writes=%llu "
      "queue_drops=%llu latency_lt2=%llu latency_2to3=%llu latency_3to5=%llu "
      "latency_5to10=%llu latency_gt10=%llu max_write=%.3fms "
      "max_submit_gap=%.3fms max_queue=%zu",
      static_cast<unsigned long long>(blocks),
      static_cast<unsigned long long>(calls),
      static_cast<unsigned long long>(shortWrites),
      static_cast<unsigned long long>(latestDrops - startingDrops),
      static_cast<unsigned long long>(under2ms),
      static_cast<unsigned long long>(from2to3ms),
      static_cast<unsigned long long>(from3to5ms),
      static_cast<unsigned long long>(from5to10ms),
      static_cast<unsigned long long>(over10ms),
      maxWriteMs, maxSubmitGapMs, maxQueueDepth);

    diagnosticActive = false;
    havePreviousWriteEnd = false;
    blocks = calls = shortWrites = 0U;
    under2ms = from2to3ms = from3to5ms = from5to10ms = over10ms = 0U;
    startingDrops = latestDrops = 0U;
    maxQueueDepth = 0U;
    maxWriteMs = maxSubmitGapMs = 0.0;
  };

  while (m_txWorkerRunning.load()) {
    TXBlock block;
    size_t queueDepth = 0U;
    uint64_t queueDrops = 0U;

    {
      std::unique_lock<std::mutex> lock(m_txQueueMutex);
      m_txQueueCondition.wait(lock, [this]() {
        return !m_txWorkerRunning.load() || !m_txQueue.empty();
      });

      if (!m_txWorkerRunning.load())
        break;

      block = std::move(m_txQueue.front());
      m_txQueue.pop_front();
      queueDepth = m_txQueue.size();
      queueDrops = m_txQueueDrops;
    }

    const bool activeBlock = std::any_of(
      block.samples.begin(), block.samples.end(),
      [](const std::complex<float>& sample) {
        return std::norm(sample) > 1.0e-12F;
      });

    if (activeBlock && !diagnosticActive) {
      diagnosticActive = true;
      startingDrops = latestDrops = queueDrops;
      LogMessage("LibreSDR TX worker diagnostic started");
    } else if (!activeBlock && diagnosticActive) {
      latestDrops = queueDrops;
      finishDiagnostic();
    }

    if (diagnosticActive) {
      blocks++;
      latestDrops = queueDrops;
      maxQueueDepth = std::max(maxQueueDepth, queueDepth);
    }

    size_t samplesWritten = 0U;
    while (samplesWritten < block.samples.size() && m_txWorkerRunning.load()) {
      void* buffs[1] = {
        static_cast<void*>(block.samples.data() + samplesWritten)
      };
      int chunkFlags = block.flags;
      long long chunkTimeNs = block.timeNs;

      if (samplesWritten > 0U) {
        chunkFlags &= ~SOAPY_SDR_HAS_TIME;
        chunkTimeNs = 0LL;
      }

      const size_t samplesRequested = block.samples.size() - samplesWritten;
      const auto writeStart = Clock::now();

      if (diagnosticActive && havePreviousWriteEnd) {
        const double submitGapMs = std::chrono::duration<double, std::milli>(
          writeStart - previousWriteEnd).count();
        maxSubmitGapMs = std::max(maxSubmitGapMs, submitGapMs);
      }

      const int ret = m_device->writeStream(
        m_txStream, buffs, samplesRequested, chunkFlags, chunkTimeNs);

      const auto writeEnd = Clock::now();
      const double writeMs = std::chrono::duration<double, std::milli>(
        writeEnd - writeStart).count();

      if (ret <= 0) {
        LogError("LibreSDR TX worker error after %zu/%zu samples: %d (%s)",
                 samplesWritten, block.samples.size(), ret, SoapySDR_errToStr(ret));
        m_txWorkerFailed.store(true);
        m_txWorkerRunning.store(false);
        break;
      }

      if (diagnosticActive) {
        calls++;
        maxWriteMs = std::max(maxWriteMs, writeMs);
        if (static_cast<size_t>(ret) < samplesRequested)
          shortWrites++;

        if (writeMs < 2.0)
          under2ms++;
        else if (writeMs < 3.0)
          from2to3ms++;
        else if (writeMs < 5.0)
          from3to5ms++;
        else if (writeMs < 10.0)
          from5to10ms++;
        else
          over10ms++;

        previousWriteEnd = writeEnd;
        havePreviousWriteEnd = true;
      }

      samplesWritten += static_cast<size_t>(ret);
    }
  }

  finishDiagnostic();
}

int CSDRSoapy::readRXSamples(RXSample* rxSamples) {
  if (m_rxBuffer.dataSize() >= RX_BLOCK_SIZE) {
    for (uint16_t i = 0U; i < RX_BLOCK_SIZE; i++) {
      m_rxBuffer.getData(*(rxSamples + i));
    }
    return RX_BLOCK_SIZE;
  }

  return 0;
}

void CSDRSoapy::process()
{
  if (!m_started)
    return;

  if (m_device == nullptr)
    return;

  assert(m_device != nullptr);
  assert(m_rxStream != nullptr);
  assert(m_txStream != nullptr);

  if (!m_soapyInit) {
    m_device->activateStream(m_rxStream);
    m_device->activateStream(m_txStream);

    startTXWorker();

    if (!m_timestamped) {
      // Write initial zeros to transmit buffer to start streams
      for (size_t i = 0; i < m_buffer.size(); i++)
        m_buffer[i] = { 0.0F, 0.0F };

      for (size_t i = 0; i < LATENCY_BLOCKS; i++) {
        if (m_useTXWorker)
          enqueueTXBlock(m_buffer);
        else if (!writeTXBlock(m_buffer))
          break;
      }
    }

    m_soapyInit = true;
  }

  if (m_useTXWorker && m_txWorkerFailed.load()) {
    LogError("LibreSDR TX worker stopped; resetting Soapy streams");
    m_soapyInit = false;
  }

  void *buffs[1] = {(void*)m_buffer.data()};
  long long timeNs = 0LL;

  if (m_soapyInit) {
    int flags = 0;
    int ret = m_device->readStream(m_rxStream, buffs, m_buffer.size(), flags, timeNs);
    if (ret > 0) {
      processIQBlock();
    } else {
      LogError("RX stream error: %d (%s)", ret, SoapySDR_errToStr(ret));
      m_soapyInit = false;
    }
  }

  if (m_soapyInit) {
    int flags = 0;
    if (m_timestamped) {
      timeNs += m_latencyNs;
      flags   = SOAPY_SDR_HAS_TIME;
    }

    if (m_useTXWorker)
      enqueueTXBlock(m_buffer, flags, timeNs);
    else if (!writeTXBlock(m_buffer, flags, timeNs))
      m_soapyInit = false;
  }

  if (!m_soapyInit) {
    stopTXWorker();
    m_device->deactivateStream(m_rxStream);
    m_device->deactivateStream(m_txStream);
    return;
  }

  // Switch off the transmitter if needed
  if (!m_txBuffer.hasData() && m_tx) {
    m_tx = false;
    LogMessage("TX OFF");

    if (m_soapyDeviceType.compare("libresdr") == 0 ||
        m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0 ||
        m_soapyDeviceType.compare("limesdr") == 0  || m_soapyDeviceType.compare("lime") == 0  ||
        m_soapyDeviceType.compare("limemini") == 0 || m_soapyDeviceType.compare("lime-mini") == 0 ||
        m_soapyDeviceType.compare("usrp") == 0)
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, 1.0);
    else
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "NONE");

    // Return to the main transmit frequency
    setTXFrequency(false);
  }

  // IO.cpp will call the getRXSamples()
}

void CSDRSoapy::processIQBlock()
{
  assert(m_fdudc != nullptr);
  assert(m_delayedTXBuffer != nullptr);

  unsigned int txSourceUnderflows = 0U;

  // Mute the receiver when transmitting in simplex mode
  if (m_tx && !m_duplex) {
    for (auto& d : m_buffer)
      d = {0.0F, 0.0F};
  }

  // Insert a channel filter here

  m_fdudc->process(m_buffer, [this, &txSourceUnderflows](std::complex<float> rxIQSample) {
    std::complex<float> txIQSample = {0.0F, 0.0F};
    TXSample txSample = {0, MARK_NONE};

    if (m_txBuffer.getData(txSample)) {
      // Modulate TX
      m_phase += txSample.m_sample * FM_DEVIATION;
      float ph = m_phase * float(M_PI / 0x80000000UL);
      txIQSample = std::polar(m_power, ph);
    } else if (m_useTXWorker && m_tx) {
      txSourceUnderflows++;
    }

    // Demodulate RX
    float d = std::arg(rxIQSample * std::conj(m_prevRXIQSample));
    m_prevRXIQSample = rxIQSample;

    // Scale -pi...pi to -4096...4096
    d *= 4096.0F / M_PI;

    txSample = m_delayedTXBuffer->process(txSample);

    float rssi = 100000000.0F * std::norm(rxIQSample);
    if (rssi > 65535.0F)
      rssi = 65535.0F;

    RXSample rxSample = {
      .m_sample  = q15_t(d + 0.5F),
      .m_rssi    = uint16_t(rssi),
      .m_control = txSample.m_control
    };

    m_rxBuffer.addData(rxSample);

    return txIQSample;
  });

  if (m_useTXWorker) {
    static bool underflowActive = false;
    static unsigned long long underflowSamples = 0ULL;
    static unsigned long long underflowBlocks = 0ULL;

    if (txSourceUnderflows > 0U) {
      underflowSamples += txSourceUnderflows;
      underflowBlocks++;

      if (!underflowActive) {
        underflowActive = true;
        LogError("LibreSDR TX source underflow started: missing=%u samples in current block",
                 txSourceUnderflows);
      }
    } else if (underflowActive) {
      LogError("LibreSDR TX source underflow ended: missing=%llu samples across %llu blocks",
               underflowSamples, underflowBlocks);
      underflowActive = false;
      underflowSamples = 0ULL;
      underflowBlocks = 0ULL;
    }
  }
}

int CSDRSoapy::read(MMDVM_STATE mode, q15_t* samples, uint16_t* rssi, uint8_t* control) {
  RXSample rxSamples[2];

  if (this->readRXSamples(rxSamples) == RX_BLOCK_SIZE) {
    for (unsigned int i = 0; i < RX_BLOCK_SIZE; i++) {
      samples[i] = rxSamples[i].m_sample;
      rssi[i] = rxSamples->m_rssi;
      control[i] = rxSamples[i].m_control;
    }

    return RX_BLOCK_SIZE;
  }

  return 0;
}

void CSDRSoapy::write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control)
{
  assert(samples != nullptr);
  assert(length > 0U);

  if (!m_started)
    return;

  if (!m_tx) {
      m_tx = true;
      LogMessage("TX ON");
      if (m_soapyDeviceType.compare("libresdr") == 0 ||
          m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0 ||
          m_soapyDeviceType.compare("limesdr") == 0  || m_soapyDeviceType.compare("lime") == 0  ||
          m_soapyDeviceType.compare("limemini") == 0 || m_soapyDeviceType.compare("lime-mini") == 0 ||
          m_soapyDeviceType.compare("usrp") == 0) {
        m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_txGain);
      } else {
        m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "TX");
      }
  }

  if (m_tx) {
    // Set the correct transmit frequency for the mode if needed, even in the middle of a transmission
    setTXFrequency(mode == MMDVM_STATE::POCSAG);
  }

  q15_t txLevel;
  switch (mode) {
    case MMDVM_STATE::FM:
      txLevel = LEVEL_100PC;
      break;
    default:
      txLevel = LEVEL_40PC_INVERTED;
      break;
  }

  for (uint16_t i = 0U; i < length; i++) {
    q31_t res1 = samples[i] * txLevel;
    q15_t res2 = q15_t(__SSAT((res1 >> 15), 16));

    if (control == nullptr)
      m_txBuffer.addData({res2, MARK_NONE});
    else
      m_txBuffer.addData({res2, control[i]});
  }
}

uint16_t CSDRSoapy::getSpace() const
{
  return m_txBuffer.freeSpace();
}

void CSDRSoapy::setTXFrequency(bool pocsag)
{
  if (m_device != nullptr) {
    if (pocsag && !m_pocsag) {
      m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyPocsagFreq);
      m_pocsag = true;
      return;
    }
        
    if (!pocsag && m_pocsag) {
      m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXFreq);
      m_pocsag = false;
      return;
    }
  }
}

uint8_t CSDRSoapy::setParameters()
{
  stop();

  if (m_trace)
    SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
  else
    SoapySDR::setLogLevel(SOAPY_SDR_INFO);

  SoapySDR::Kwargs devArgs;
  SoapySDR::Kwargs rxArgs;
  SoapySDR::Kwargs txArgs;

  // Default FDUDC parameters running 150k SPS
  unsigned int resampNum = 4U;
  unsigned int resampDen = 25U;
  size_t blockSize = 512U;
  size_t iqHWDelay = 10;
  float cutoff = 0.5F;

  const unsigned int resampLen = 11U;
  const unsigned int rxIfNum = 1U, rxIfDen = 12U;
  const unsigned int txIfNum = 1U, txIfDen = 12U;

  const char* PLUTO_DEFAULT_URI = "ip:pluto.local";
  const char* LIME_DEFAULT_URI  = "index=0";         // eg: addr=1111:2222 or serial=xxxxxxxx

  m_useTXWorker = false;

  if (m_soapyDeviceType.compare("libresdr") == 0) {
    const char* uri = m_soapyDeviceURI.empty() ? PLUTO_DEFAULT_URI : m_soapyDeviceURI.c_str();

    // LibreSDR - running 1.2M SPS
    resampNum = 1U;
    resampDen = 50U;
    blockSize = 4096U;
    iqHWDelay = 10U;
    cutoff = 0.25F;

    devArgs["driver"] = "plutosdr";
    rxArgs["uri"]     = uri;

    m_timestamped = false;
    m_useTXWorker = true;

    LogMessage("Using LibreSDR profile with Pluto SDR driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0) {
    const char* uri = m_soapyDeviceURI.empty() ? PLUTO_DEFAULT_URI : m_soapyDeviceURI.c_str();

    // PlutoSDR - running 300k SPS
    resampNum = 2U;
    resampDen = 25U;
    blockSize = 1024U;
    iqHWDelay = 10U;
    cutoff = 0.25F;

    devArgs["driver"] = "plutosdr";
    rxArgs["uri"]     = uri;

    m_timestamped = false;

    LogMessage("Using Pluto SDR driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("limesdr") == 0 || m_soapyDeviceType.compare("lime") == 0) {
    const char* uri = m_soapyDeviceURI.empty() ? LIME_DEFAULT_URI : m_soapyDeviceURI.c_str();

    // LimeSDR Legacy - running 300k SPS
    resampNum = 2U;
    resampDen = 25U;
    blockSize = 1024U;
    iqHWDelay = 10U;
    cutoff = 0.25F;

    devArgs["driver"] = "lime";
    rxArgs["uri"]     = uri;
    rxArgs["latency"] = "0";
    txArgs["latency"] = "0";

    m_timestamped = true;

    LogMessage("Using Lime SDR driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("limemini") == 0 || m_soapyDeviceType.compare("limenet-micro") == 0) {
    const char* uri = m_soapyDeviceURI.empty() ? LIME_DEFAULT_URI : m_soapyDeviceURI.c_str();

    resampNum = 2U;
    resampDen = 50U;
    blockSize = 2048U;
    iqHWDelay = 50U;

    devArgs["driver"] = "lime";
    rxArgs["uri"]     = uri;
    rxArgs["latency"] = "0";
    txArgs["latency"] = "0";

    m_timestamped = true;

    LogMessage("Using LimeSDR-mini driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("usrp") == 0) {
    const char* uri = m_soapyDeviceURI.c_str();

    resampNum = 2U;
    resampDen = 50U;
    blockSize = 2048U;
    iqHWDelay = 50U;

    devArgs["driver"] = "uhd";
    rxArgs["uri"]     = uri;
    txArgs["uri"]     = uri;
    rxArgs["recv_frame_size"] = "1024";

    m_timestamped = true;

    LogMessage("Using Ettus USRP driver uri %s", uri);
  } else if (m_soapyDeviceType.compare("mucell") == 0) {
    // mucell(sx1255) running 150k SPS
    resampNum = 4U;
    resampDen = 25U;
    blockSize = 512U;
    iqHWDelay = 10U;
    cutoff = 0.5F;

    devArgs["driver"] = "mucell";

    m_timestamped = true;

    LogMessage("Using muCell driver");
  } else {
    // sxceiver(sx1255) running 150k SPS (default)
    resampNum = 4U;
    resampDen = 25U;
    blockSize = 512U;
    iqHWDelay = 10U;
    cutoff = 0.5F;

    devArgs["driver"] = "sx";

    m_timestamped = true;

    LogMessage("Using SX1255 driver");
  }

  m_buffer.resize(blockSize);

  size_t latencySamples = (blockSize * LATENCY_BLOCKS + iqHWDelay) * resampNum / resampDen + resampLen;

  m_delayedTXBuffer = new CDelayBuffer<TXSample>(latencySamples, {0, 0U});

  const double samplerate = 24000.0 * double(resampDen) / double(resampNum);

  m_latencyNs = (long long)std::round(1e9 / samplerate * (double)(blockSize * LATENCY_BLOCKS));

  m_fdudc = new CFDUDC(resampNum, resampDen, rxIfNum, rxIfDen, txIfNum, txIfDen, resampLen, cutoff);

  m_soapyTXFreq     = double(m_txFreq)     - samplerate * double(txIfNum) / double(txIfDen);
  m_soapyPocsagFreq = double(m_pocsagFreq) - samplerate * double(txIfNum) / double(txIfDen);
  m_soapyRXFreq     = double(m_rxFreq)     - samplerate * double(rxIfNum) / double(rxIfDen);

  LogMessage("SDR Parameters");
  LogMessage("  Sample Rate:      %.0f samples/sec", samplerate);
  LogMessage("  Latency    :      %.2f ms", (double)m_latencyNs / 1e6);
  LogMessage("  TX Frequency:     %.0f Hz", m_soapyTXFreq);
  LogMessage("  RX Frequency:     %.0f Hz", m_soapyRXFreq);
  LogMessage("  POCSAG Frequency: %.0f Hz", m_soapyPocsagFreq);

  try {
    m_device = SoapySDR::Device::make(devArgs);
    assert(m_device != nullptr);

    m_device->setSampleRate(SOAPY_SDR_RX, RX_CHANNEL, samplerate);
    m_device->setSampleRate(SOAPY_SDR_TX, TX_CHANNEL, samplerate);

    LogMessage("  Actual RX Rate:   %.0f samples/sec",
               m_device->getSampleRate(SOAPY_SDR_RX, RX_CHANNEL));
    LogMessage("  Actual TX Rate:   %.0f samples/sec",
               m_device->getSampleRate(SOAPY_SDR_TX, TX_CHANNEL));

    m_device->setFrequency(SOAPY_SDR_RX, RX_CHANNEL, m_soapyRXFreq);
    m_device->setFrequency(SOAPY_SDR_TX, TX_CHANNEL, m_soapyTXFreq);

    if (m_soapyDeviceType.compare("libresdr") == 0 ||
        m_soapyDeviceType.compare("plutosdr") == 0 || m_soapyDeviceType.compare("pluto") == 0) {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "A_BALANCED");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "A");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, m_rxGain);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_txGain);
    } else if (m_soapyDeviceType.compare("limesdr") == 0 || m_soapyDeviceType.compare("lime") == 0) {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "LNAH");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "BAND1");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, m_rxGain);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_txGain);
    } else if (m_soapyDeviceType.compare("limemini") == 0 || m_soapyDeviceType.compare("lime-mini") == 0) {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "Auto");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "Auto");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, m_rxGain);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_txGain);
    } else if (m_soapyDeviceType.compare("usrp") == 0) {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "RX2");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "TX/RX");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, m_rxGain);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_txGain);
    } else {
      m_device->setAntenna(SOAPY_SDR_RX, RX_CHANNEL, "RX");
      m_device->setAntenna(SOAPY_SDR_TX, TX_CHANNEL, "NONE");

      m_device->setGain(SOAPY_SDR_RX, RX_CHANNEL, m_rxGain);
      m_device->setGain(SOAPY_SDR_TX, TX_CHANNEL, m_txGain);
    }

    m_rxStream = m_device->setupStream(SOAPY_SDR_RX, "CF32", {RX_CHANNEL}, rxArgs);
    m_txStream = m_device->setupStream(SOAPY_SDR_TX, "CF32", {TX_CHANNEL}, txArgs);

    assert(m_rxStream != nullptr);
    assert(m_txStream != nullptr);

    m_soapyInit = false;

    LogMessage("SoapySDR device setup done");
  } catch (std::runtime_error &e) {
    LogError("Error setting up SoapySDR device: %s", e.what());
    return 4U;
  }

  return 0U;
}

void CSDRSoapy::setDeviceInfo(const std::string& type, const std::string& uri, unsigned int rxGain, unsigned int txGain)
{
  m_soapyDeviceType = type;
  m_soapyDeviceURI  = uri;

  m_rxGain = float(rxGain);
  m_txGain = float(txGain);
}

uint8_t CSDRSoapy::setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
  if ((txFreq < MIN_RF_FREQUENCY) || (txFreq > MAX_RF_FREQUENCY))
    return 4U;

  if ((rxFreq < MIN_RF_FREQUENCY) || (rxFreq > MAX_RF_FREQUENCY))
    return 4U;

  if ((pocsagFreq < MIN_RF_FREQUENCY) || (pocsagFreq > MAX_RF_FREQUENCY))
    return 4U;

  m_power      = float(power) / 255.0F;
  m_txFreq     = txFreq;
  m_rxFreq     = rxFreq;
  m_pocsagFreq = pocsagFreq;

  return 0U;
}

#endif
