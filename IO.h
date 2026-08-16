/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
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

#if !defined(IO_H)
#define  IO_H

#include "DelayBuffer.h"
#include "RingBuffer.h"
#include "Socket.h"
#include "FDUDC.h"

#include <vector>

class ISDRDevice;

struct TXSample {
  volatile q15_t   m_sample;
  volatile uint8_t m_control;
};

struct RXSample {
  volatile q15_t    m_sample;
  volatile uint16_t m_rssi;
  volatile uint8_t  m_control;
};

class CIO {
public:
  CIO();
  ~CIO();

  bool start(bool trace);

  void process(bool networkData=false);

  void processMultiNetwork() {};

  void stop();

  void write(MMDVM_STATE mode, const q15_t* samples, uint16_t length, const uint8_t* control = nullptr);

  uint16_t getSpace() const;

  void setMode(MMDVM_STATE state);

  void setSoapyDeviceInfo(const std::string& type, const std::string& uri, unsigned int rxGain, unsigned int txGain);
  void setMultiModemAddress(std::string myAddress, unsigned short myPort, std::string modemAddress, unsigned short modemPort);
  
  uint8_t setFrequency(uint8_t power, uint32_t txFreq, uint32_t rxFreq, uint32_t pocsagFreq);
  uint8_t setParameters();

private:
  ISDRDevice*           m_sdrDevice;
  bool                  m_trace;
  bool                  m_started;

#if defined(USE_DCBLOCKER)
  arm_biquad_casd_df1_inst_q31 m_dcFilter;
  q31_t                        m_dcState[4];
#endif

#if defined(MODE_DSTAR)
  arm_fir_instance_q15 m_gaussianFilter;
  q15_t                m_gaussianState[40U];      // NoTaps + BlockSize - 1, 12 + 20 - 1 plus some spare
#endif

#if defined(MODE_DMR)
  arm_fir_instance_q15 m_rrc02Filter1;
  q15_t                m_rrc02State1[70U];         // NoTaps + BlockSize - 1, 42 + 20 - 1 plus some spare
#endif

#if defined(MODE_YSF)
  arm_fir_instance_q15 m_rrc02Filter2;
  q15_t                m_rrc02State2[70U];         // NoTaps + BlockSize - 1, 42 + 20 - 1 plus some spare
#endif

#if defined(MODE_P25)
  arm_fir_instance_q15 m_boxcar5Filter;
  q15_t                m_boxcar5State[30U];        // NoTaps + BlockSize - 1,  6 + 20 - 1 plus some spare
#endif

#if defined(MODE_NXDN)
#if defined(USE_NXDN_BOXCAR)
  arm_fir_instance_q15 m_boxcar10Filter;
  q15_t                m_boxcar10State[40U];      // NoTaps + BlockSize - 1, 10 + 20 - 1 plus some spare
#else
  arm_fir_instance_q15 m_nxdnFilter;
  arm_fir_instance_q15 m_nxdnISincFilter;
  q15_t                m_nxdnState[110U];         // NoTaps + BlockSize - 1, 82 + 20 - 1 plus some spare
  q15_t                m_nxdnISincState[60U];     // NoTaps + BlockSize - 1, 32 + 20 - 1 plus some spare
#endif
#endif

  float                m_power;
  uint32_t             m_txFreq;
  uint32_t             m_rxFreq;
  uint32_t             m_pocsagFreq;
  float                m_rxGain;
  float                m_txGain;
};

#endif
