/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
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

#include "Globals.h"
#include "Config.h"

#include "SDRMulti.h"

#include <cstdio>
#include <cassert>

// To allow for fine tuning of the deviation levels
// const q15_t LEVEL_50PC_INVERTED = -128 * 128;
const q15_t LEVEL_40PC_INVERTED = -102 * 128;
// const q15_t LEVEL_30PC_INVERTED = -77  * 128;
const q15_t LEVEL_100PC         =  255 * 128;

const unsigned int SAMPLES_TO_NETWORK = 720U;
const unsigned int MULTIMODEM_PACKET_SIZE = SAMPLES_TO_NETWORK * 3U + 8U;

CSDRMulti::CSDRMulti() :
m_trace(false),
m_rxNetworkBuffer(721U, "MMDVM-Multi RX Buffer"),
m_txNetworkBuffer(722U, "MMDVM-Multi TX Buffer"),
m_multiModemSocket(),
m_myAddress("127.0.0.1"),
m_myPort(48100),
m_modemAddress("127.0.0.1"),
m_modemPort(48200),
m_txTimeout(1000, 0, 250)
{
}

CSDRMulti::~CSDRMulti()
{
}

void CSDRMulti::setAddress(std::string myAddress, unsigned short myPort, std::string modemAddress, unsigned short modemPort) {
  m_myAddress = myAddress;
  m_myPort = myPort;
  m_modemAddress = modemAddress;
  m_modemPort = modemPort;
}

bool CSDRMulti::start(bool trace)
{
  m_trace = trace;

  assert(m_myPort > 0);
  assert(m_modemPort > 0);

  LogDebug("Opening SDRMulti socket my(%u) <--> multi(%u)", m_myPort, m_modemPort);
  bool res = m_multiModemSocket.open(m_myAddress, m_myPort, m_modemAddress, m_modemPort);
  if (!res)
    return false;

  LogMessage("SDRMulti started");

  return true;
}

void CSDRMulti::stop()
{
  m_multiModemSocket.close();
  LogMessage("SDRMulti stopped");
}

void CSDRMulti::process()
{
  // TX flag reset timer
  m_txTimeout.clock();
  if (m_tx && m_txTimeout.hasExpired()) {
    m_txTimeout.stop();
    m_tx = false;
  }

  if (m_txNetworkBuffer.hasData() && !m_tx) {
    LogMessage("TX OFF");
    m_txNetworkBuffer.clear();  // clear off partial DMR timeslot data so good timing info is present in packet
  }

  if (!m_txNetworkBuffer.hasData() && m_tx) {
    m_tx = false;
    LogMessage("TX OFF");
  }

  // Keep the trace code for later debug
  if (m_trace) {
    unsigned int ds = m_txNetworkBuffer.dataSize();
    if (ds > 0)
      LogDebug("SDRMulti - tx buffered data remaining %u", ds);
  }

  uint32_t num_send_items = SAMPLES_TO_NETWORK;
  unsigned char recv_message[MULTIMODEM_PACKET_SIZE];
  ::memset(recv_message, 0U, MULTIMODEM_PACKET_SIZE);

  int num_bytes = m_multiModemSocket.readDatagram(recv_message, MULTIMODEM_PACKET_SIZE);
  if (num_bytes == MULTIMODEM_PACKET_SIZE) {
    if ((m_txNetworkBuffer.hasData()) && (m_txNetworkBuffer.dataSize() >= num_send_items)) {
      m_txTimeout.start();

      TXSample samples[SAMPLES_TO_NETWORK];
      m_txNetworkBuffer.getData(samples, num_send_items);
      unsigned char reply[MULTIMODEM_PACKET_SIZE];
      ::memset(reply, 0U, MULTIMODEM_PACKET_SIZE);
      ::memcpy(reply, &num_send_items, sizeof(uint32_t));

      for (unsigned int i = 0U; i < num_send_items; i++) {
        int16_t sample = samples[i].m_sample;
        uint8_t control = samples[i].m_control;
        ::memcpy(reply + sizeof(uint32_t) + i * sizeof(uint8_t), &control, sizeof(uint8_t));
        ::memcpy(reply + sizeof(uint32_t) + num_send_items * sizeof(uint8_t) + i * sizeof(int16_t),
                &sample, sizeof(int16_t));
      }

      if (!m_multiModemSocket.write(reply, MULTIMODEM_PACKET_SIZE - 4U))
        LogError("Error writing to socket\n");
    } else {
      unsigned char reply[4U];
      ::memset(reply, 0U, 4U);

      if (!m_multiModemSocket.write(reply, 4U))
        LogError("Error writing to socket\n");
    }

    uint32_t data_size = 0;
    ::memcpy(&data_size, recv_message, sizeof(uint32_t));

    if (data_size != SAMPLES_TO_NETWORK) {
      LogError("Received malformed packet from MMDVM-Multi: %u samples", data_size);
      return;
    }

    uint32_t rssi = 0;
    ::memcpy(&rssi, recv_message + sizeof(uint32_t), sizeof(uint32_t));

    for (uint32_t i = 0U; i < data_size; i++) {
      int16_t sample = 0;
      uint8_t control = MARK_NONE;
      ::memcpy(&control, recv_message + 2U * sizeof(uint32_t) + i, sizeof(uint8_t));
      ::memcpy(&sample, recv_message + 2U * sizeof(uint32_t) + data_size * sizeof(uint8_t) + i * sizeof(int16_t), sizeof(int16_t));
      RXSample rx_sample;
      rx_sample.m_sample = sample;
      rx_sample.m_control = control;
      rx_sample.m_rssi = (uint16_t)rssi;
      m_rxNetworkBuffer.addData(rx_sample);
    }
  }
}

int CSDRMulti::read(MMDVM_STATE mode, q15_t* samples, uint16_t* rssi, uint8_t* control) {
  if (m_rxNetworkBuffer.dataSize() >= RX_BLOCK_SIZE) {
    for (unsigned int i = 0; i < RX_BLOCK_SIZE; i++) {
      RXSample rxSample{};
      m_rxNetworkBuffer.getData(rxSample);
      samples[i] = rxSample.m_sample;
      rssi[i] = rxSample.m_rssi;
      control[i] = rxSample.m_control;
    }

    return RX_BLOCK_SIZE;
  }

  return 0;
}

void CSDRMulti::write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control)
{
  assert(samples != nullptr);
  assert(length > 0U);

  if (!m_tx) {
      m_tx = true;
      LogMessage("TX ON");

      m_txTimeout.start();
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
      m_txNetworkBuffer.addData({res2, MARK_NONE});
    else
      m_txNetworkBuffer.addData({res2, control[i]});
  }

  // LogDebug("SDRMulti - write %u mode data %u, space left %u", (int)mode, length, m_txNetworkBuffer.freeSpace());
}

uint16_t CSDRMulti::getSpace() const
{
  return m_txNetworkBuffer.freeSpace();
}

uint8_t CSDRMulti::setParameters()
{
  return 0U;
}

uint8_t CSDRMulti::setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq)
{
  (void)power;
  (void)txFreq;
  (void)rxFreq;
  (void)pocsagFreq;
  return 0U;
}
