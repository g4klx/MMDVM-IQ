/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2023 by Tatu Peltola OH2EAT
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
 *   Copyright (C) 2026 by Adrian Musceac YO8RZZ
 *   Copyright (C) 2026 by Shawn Chain BG5HHP
 *   Copyright (C) 2026 by Steve Miller KC1AWV
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

#include <cstdio>
#include <cassert>

#if !defined(M_PI)
#define M_PI    3.141592654
#endif

const size_t RX_CHANNEL = 0;
const size_t TX_CHANNEL = 0;

const size_t LATENCY_BLOCKS = 3;

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

    if (!m_timestamped) {
      // Write initial zeros to transmit buffer to start streams
      for (size_t i = 0; i < m_buffer.size(); i++)
        m_buffer[i] = { 0.0F, 0.0F };

      for (size_t i = 0; i < LATENCY_BLOCKS; i++) {
        if (!writeTXBlock(m_buffer))
          break;
      }
    }

    m_soapyInit = true;
  }

  void *buffs[1] = {(void*)m_buffer.data()};
  long long timeNs = 0LL;

  if (m_soapyInit) {
    if (m_soapyDeviceType.compare("usrp") == 0) {
      // Local testing has shown the N210 delivers samples in smaller Ethernet
      // packets. Collect multiple packets into one larger application buffer
      // before processing and transmitting. This reduces processing overhead
      // and prevents unfilled parts of the buffer from being treated as new
      // sample data.
      size_t samplesRead = 0U;

      while (samplesRead < m_buffer.size() && m_soapyInit) {
        void* rxBuffs[1] = {m_buffer.data() + samplesRead};
        int flags = 0;
        const int ret = m_device->readStream(
          m_rxStream, rxBuffs, m_buffer.size() - samplesRead, flags, timeNs);

        if (ret > 0) {
          samplesRead += static_cast<size_t>(ret);
        } else {
          LogError("RX stream error after %zu/%zu samples: %d (%s)",
                   samplesRead, m_buffer.size(), ret, SoapySDR_errToStr(ret));
          m_soapyInit = false;
        }
      }

      if (m_soapyInit)
        processIQBlock();
    } else {
      int flags = 0;
      const int ret = m_device->readStream(
        m_rxStream, buffs, m_buffer.size(), flags, timeNs);

      if (ret > 0) {
        processIQBlock();
      } else {
        LogError("RX stream error: %d (%s)", ret, SoapySDR_errToStr(ret));
        m_soapyInit = false;
      }
    }
  }

  if (m_soapyInit) {
    int flags = 0;
    if (m_timestamped) {
      timeNs += m_latencyNs;
      flags   = SOAPY_SDR_HAS_TIME;
    }

    if (!writeTXBlock(m_buffer, flags, timeNs))
      m_soapyInit = false;
  }

  if (!m_soapyInit) {
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

  // Mute the receiver when transmitting in simplex mode
  if (m_tx && !m_duplex) {
    for (auto& d : m_buffer)
      d = {0.0F, 0.0F};
  }

  // Insert a channel filter here

  m_fdudc->process(m_buffer, [this](std::complex<float> rxIQSample) {
    std::complex<float> txIQSample = {0.0F, 0.0F};
    TXSample txSample = {0, MARK_NONE};

    if (m_txBuffer.getData(txSample)) {
      // Modulate TX
      m_phase += txSample.m_sample * FM_DEVIATION;
      float ph = m_phase * float(M_PI / 0x80000000UL);
      txIQSample = std::polar(m_power, ph);
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
    rxArgs["bufflen"] = std::to_string(blockSize);
    txArgs["bufflen"] = std::to_string(blockSize);

    m_timestamped = false;

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
    
    // Ettus USRP - running 1M SPS
    resampNum = 3U;
    resampDen = 125U;
    blockSize = 4096U;
    iqHWDelay = 50U;
    cutoff = 0.45F;

    devArgs["driver"] = "uhd";
    
    if (!m_soapyDeviceURI.empty()) {
      const SoapySDR::Kwargs uriArgs = SoapySDR::KwargsFromString(m_soapyDeviceURI);
      devArgs.insert(uriArgs.begin(), uriArgs.end());
    }

    m_timestamped = true;

    LogMessage("Using Ettus USRP driver args %s", m_soapyDeviceURI.empty() ? "(default)" : m_soapyDeviceURI.c_str());
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

    LogMessage("  RX stream MTU:    %zu samples", m_device->getStreamMTU(m_rxStream));
    LogMessage("  TX stream MTU:    %zu samples", m_device->getStreamMTU(m_txStream));

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
