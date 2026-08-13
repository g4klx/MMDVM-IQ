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

#if !defined(IOSDR_UDP_H)
#define  IOSDR_UDP_H

#include "IO.h"
#include "SDRDevice.h"
#include "Timer.h"

#include <vector>
#include <string>

class CSDRMulti : public ISDRDevice {
public:
  CSDRMulti();
  virtual ~CSDRMulti();

  bool start(bool trace);

  void process();

  void stop();

  void setAddress(std::string myAddress, unsigned short myPort, std::string modemAddress, unsigned short modemPort);

  void write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control = NULL);
  int read(MMDVM_STATE mode, q15_t* samples, uint16_t* rssi, uint8_t* control);

  uint16_t getSpace() const;

  void setDeviceInfo(const std::string& type, const std::string& uri, unsigned int rxGain, unsigned int txGain);
  
  uint8_t setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);
  uint8_t setParameters();

private:
  bool                  m_trace = false;
  CRingBuffer<RXSample> m_rxNetworkBuffer;
  CRingBuffer<TXSample> m_txNetworkBuffer;

  CSocket              m_multiModemSocket;

  std::string          m_myAddress;
  unsigned short       m_myPort;
  std::string          m_modemAddress;
  unsigned short       m_modemPort;

  CTimer               m_txTimeout;
};

#endif
