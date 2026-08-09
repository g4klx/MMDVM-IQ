/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
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

#if !defined(SDR_SOAPY_H)
#define  SDR_SOAPY_H

#if defined(USE_SOAPY)

#include "IO.h"
#include "SDRDevice.h"

#include "DelayBuffer.h"
#include "RingBuffer.h"
#include "Socket.h"
#include "FDUDC.h"

#include <cstdint>
#include <vector>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>

class CSDRSoapy : public ISDRDevice {
public:
  CSDRSoapy();
  virtual ~CSDRSoapy();

  bool start(bool trace);

  void process();

  void stop();

  void write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control = NULL);
  int read(MMDVM_STATE mode, q15_t* samples, uint16_t* rssi, uint8_t* control);

  int readRXSamples(RXSample* rxSamples);

  uint16_t getSpace() const;

  void setDeviceInfo(const std::string& type, const std::string& uri, unsigned int rxGain, unsigned int txGain);
  
  uint8_t setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);
  uint8_t setParameters();

private:
  bool                  m_trace = false;

  bool                  m_started = false;
  CRingBuffer<RXSample> m_rxBuffer;
  CRingBuffer<TXSample> m_txBuffer;


  float                m_power;
  uint32_t             m_txFreq;
  uint32_t             m_rxFreq;
  uint32_t             m_pocsagFreq;
  float                m_rxGain;
  float                m_txGain;

  // Frequencies as used by SoapySDR
  double               m_soapyTXFreq;
  double               m_soapyRXFreq;
  double               m_soapyPocsagFreq;

  bool                 m_soapyInit;

  bool                 m_timestamped;
  long long            m_latencyNs;

  uint32_t             m_phase;
  std::complex<float>  m_prevRXIQSample;
  CDelayBuffer<TXSample>* m_delayedTXBuffer;

  std::vector<std::complex<float>> m_buffer;

  CFDUDC*              m_fdudc;

  std::string          m_soapyDeviceType;
  std::string          m_soapyDeviceURI;

  SoapySDR::Device*    m_device;
  SoapySDR::Stream*    m_rxStream;
  SoapySDR::Stream*    m_txStream;

  bool                 m_pocsag;

  void processIQBlock();
  bool writeTXBlock(const std::vector<std::complex<float>>& samples, int flags = 0, long long timeNs = 0LL);
  void setTXFrequency(bool pocsag);
};

#endif

#endif
