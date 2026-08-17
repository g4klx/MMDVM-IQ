/*
 *   Copyright (C) 2013,2015-2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Colin Durbridge G4EML
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

#include "Config.h"
#include "Globals.h"

#if defined(MADEBYMAKEFILE)
#include "GitVersion.h"
#endif

#include "SerialPort.h"
#include "Version.h"

#include <cassert>

const uint8_t MMDVM_FRAME_START  = 0xE0U;

const uint8_t MMDVM_GET_VERSION  = 0x00U;
const uint8_t MMDVM_GET_STATUS   = 0x01U;
const uint8_t MMDVM_SET_CONFIG   = 0x02U;
const uint8_t MMDVM_SET_MODE     = 0x03U;
const uint8_t MMDVM_SET_FREQ     = 0x04U;

const uint8_t MMDVM_CAL_DATA     = 0x08U;
const uint8_t MMDVM_RSSI_DATA    = 0x09U;

const uint8_t MMDVM_SEND_CWID    = 0x0AU;

const uint8_t MMDVM_DSTAR_HEADER = 0x10U;
const uint8_t MMDVM_DSTAR_DATA   = 0x11U;
const uint8_t MMDVM_DSTAR_LOST   = 0x12U;
const uint8_t MMDVM_DSTAR_EOT    = 0x13U;

const uint8_t MMDVM_DMR_ALOHA    = 0x17U; 
const uint8_t MMDVM_DMR_DATA1    = 0x18U;
const uint8_t MMDVM_DMR_LOST1    = 0x19U;
const uint8_t MMDVM_DMR_DATA2    = 0x1AU;
const uint8_t MMDVM_DMR_LOST2    = 0x1BU;
const uint8_t MMDVM_DMR_SHORTLC  = 0x1CU;
const uint8_t MMDVM_DMR_START    = 0x1DU;
const uint8_t MMDVM_DMR_ABORT    = 0x1EU;

const uint8_t MMDVM_YSF_DATA     = 0x20U;
const uint8_t MMDVM_YSF_LOST     = 0x21U;

const uint8_t MMDVM_P25_HDR      = 0x30U;
const uint8_t MMDVM_P25_LDU      = 0x31U;
const uint8_t MMDVM_P25_LOST     = 0x32U;

const uint8_t MMDVM_NXDN_DATA    = 0x40U;
const uint8_t MMDVM_NXDN_LOST    = 0x41U;

const uint8_t MMDVM_POCSAG_DATA  = 0x50U;

const uint8_t MMDVM_FM_PARAMS1   = 0x60U;
const uint8_t MMDVM_FM_PARAMS2   = 0x61U;
const uint8_t MMDVM_FM_PARAMS3   = 0x62U;
const uint8_t MMDVM_FM_PARAMS4   = 0x63U;
const uint8_t MMDVM_FM_DATA      = 0x65U;
const uint8_t MMDVM_FM_STATUS    = 0x66U;
const uint8_t MMDVM_FM_EOT       = 0x67U;
const uint8_t MMDVM_FM_RSSI      = 0x68U;

const uint8_t MMDVM_ACK          = 0x70U;
const uint8_t MMDVM_NAK          = 0x7FU;

const uint8_t MMDVM_SERIAL_DATA  = 0x80U;
const uint8_t MMDVM_I2C_DATA     = 0x81U;

const uint8_t MMDVM_TRANSPARENT  = 0x90U;
const uint8_t MMDVM_QSO_INFO     = 0x91U;

const uint8_t MMDVM_DEBUG1       = 0xF1U;
const uint8_t MMDVM_DEBUG2       = 0xF2U;
const uint8_t MMDVM_DEBUG3       = 0xF3U;
const uint8_t MMDVM_DEBUG4       = 0xF4U;
const uint8_t MMDVM_DEBUG5       = 0xF5U;
const uint8_t MMDVM_DEBUG_DUMP   = 0xFAU;

#define	HW_TYPE	"MMDVM-IQ"

#if defined(GITVERSION)
#define concat(h, a, b) h " " a " GitID #" b ""
const char HARDWARE[] = concat(HW_TYPE, VERSION, GITVERSION);
#else
#define concat(h, a, b, c) h " " a " (Build: " b " " c ")"
const char HARDWARE[] = concat(HW_TYPE, VERSION, __TIME__, __DATE__);
#endif

const uint8_t PROTOCOL_VERSION   = 2U;

// Parameters for batching serial data
const int      MAX_SERIAL_DATA  = 250;
const uint16_t MAX_SERIAL_COUNT = 100U;

CSerialPort::CSerialPort() :
m_buffer(),
m_ptr(0U),
m_len(0U),
m_socket(),
m_trace(false)
{
}

CSerialPort::~CSerialPort()
{
}

void CSerialPort::sendACK(uint8_t type)
{
  uint8_t reply[4U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 4U;
  reply[2U] = MMDVM_ACK;
  reply[3U] = type;

  m_socket.write(reply, 4U);

  if (m_trace)
      dump("Send ACK", reply, 4U);
}

void CSerialPort::sendNAK(uint8_t type, uint8_t err)
{
  uint8_t reply[5U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 5U;
  reply[2U] = MMDVM_NAK;
  reply[3U] = type;
  reply[4U] = err;

  m_socket.write(reply, 5U);

  if (m_trace)
      dump("Send NAK", reply, 5U);
}

void CSerialPort::getStatus()
{
  uint8_t reply[30U];

  // Send all sorts of interesting internal values
  reply[0U]  = MMDVM_FRAME_START;
  reply[1U]  = 20U;
  reply[2U]  = MMDVM_GET_STATUS;

  reply[3U]  = uint8_t(m_modemState);

  reply[4U]  = m_tx ? 0x01U : 0x00U;

  reply[4U] |= m_dcd ? 0x40U : 0x00U;

  reply[5U] = 0x00U;

#if defined(MODE_DSTAR)
  if (m_dstarEnable)
    reply[6U] = dstarTX.getSpace();
  else
    reply[6U] = 0U;
#else
  reply[6U] = 0U;
#endif

#if defined(MODE_DMR)
  if (m_dmrEnable) {
    if (m_duplex) {
      reply[7U] = dmrTX.getSpace1();
      reply[8U] = dmrTX.getSpace2();
    } else {
      reply[7U] = 10U;
      reply[8U] = dmrDMOTX.getSpace();
    }
  } else {
    reply[7U] = 0U;
    reply[8U] = 0U;
  }
#else
  reply[7U] = 0U;
  reply[8U] = 0U;
#endif

#if defined(MODE_YSF)
  if (m_ysfEnable)
    reply[9U] = ysfTX.getSpace();
  else
    reply[9U] = 0U;
#else
  reply[9U] = 0U;
#endif

#if defined(MODE_P25)
  if (m_p25Enable)
    reply[10U] = p25TX.getSpace();
  else
    reply[10U] = 0U;
#else
  reply[10U] = 0U;
#endif

#if defined(MODE_NXDN)
  if (m_nxdnEnable)
    reply[11U] = nxdnTX.getSpace();
  else
    reply[11U] = 0U;
#else
  reply[11U] = 0U;
#endif

  reply[12U] = 0U;

#if defined(MODE_FM)
  if (m_fmEnable)
    reply[13U] = fm.getSpace();
  else
    reply[13U] = 0U;
#else
  reply[13U] = 0U;
#endif

#if defined(MODE_POCSAG)
  if (m_pocsagEnable)
    reply[14U] = pocsagTX.getSpace();
  else
    reply[14U] = 0U;
#else
  reply[14U] = 0U;
#endif

  reply[15U] = 0U;

  reply[16U] = 0x00U;
  reply[17U] = 0x00U;
  reply[18U] = 0x00U;
  reply[19U] = 0x00U;

  m_socket.write(reply, 20U);

  if (m_trace)
      dump("Get Status", reply, 20U);
}

void CSerialPort::getVersion()
{
  uint8_t reply[200U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_GET_VERSION;

  reply[3U] = PROTOCOL_VERSION;

  // Return two bytes of mode capabilities
  reply[4U] = 0x00U;
#if defined(MODE_DSTAR)
  reply[4U] |= 0x01U;
#endif
#if defined(MODE_DMR)
  reply[4U] |= 0x02U;
#endif
#if defined(MODE_YSF)
  reply[4U] |= 0x04U;
#endif
#if defined(MODE_P25)
  reply[4U] |= 0x08U;
#endif
#if defined(MODE_NXDN)
  reply[4U] |= 0x10U;
#endif
#if defined(MODE_FM)
  reply[4U] |= 0x40U;
#endif

  reply[5U] = 0x00U;
#if defined(MODE_POCSAG)
  reply[5U] |= 0x01U;
#endif

  // CPU type/manufacturer. 0=Atmel ARM, 1=NXP ARM, 2=St-Micro ARM, 99=Unknown
  reply[6U] = 99U;

  uint8_t count = 7U;
  for (uint8_t i = 0U; HARDWARE[i] != 0x00U; i++, count++)
    reply[count] = HARDWARE[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Get Version", reply, count);
}

uint8_t CSerialPort::setFrequency(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 14U)
    return 4U;

  uint32_t rxFreq = 0U;
  rxFreq |= (data[1U] << 0)  & 0x990000FFU;
  rxFreq |= (data[2U] << 8)  & 0x0000FF00U;
  rxFreq |= (data[3U] << 16) & 0x00FF0000U;
  rxFreq |= (data[4U] << 24) & 0xFF000000U;

  uint32_t txFreq = 0U;
  txFreq |= (data[5U] << 0)  & 0x000000FFU;
  txFreq |= (data[6U] << 8)  & 0x0000FF00U;
  txFreq |= (data[7U] << 16) & 0x00FF0000U;
  txFreq |= (data[8U] << 24) & 0xFF000000U;

  uint8_t power = data[9U];

  uint32_t pocsagFreq = 0U;
  pocsagFreq |= (data[10U] << 0)  & 0x000000FFU;
  pocsagFreq |= (data[11U] << 8)  & 0x0000FF00U;
  pocsagFreq |= (data[12U] << 16) & 0x00FF0000U;
  pocsagFreq |= (data[13U] << 24) & 0xFF000000U;

  return io.setFrequency(power, txFreq, rxFreq, pocsagFreq);
}

uint8_t CSerialPort::setConfig(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 37U)
    return 4U;

#if defined(MODE_YSF)
  bool ysfLoDev        = (data[0U] & 0x08U) == 0x08U;
#endif
  bool simplex         = (data[0U] & 0x80U) == 0x80U;

#if defined(MODE_DSTAR)
  bool dstarEnable  = (data[1U] & 0x01U) == 0x01U;
#endif
#if defined(MODE_DMR)
  bool dmrEnable    = (data[1U] & 0x02U) == 0x02U;
#endif
#if defined(MODE_YSF)
  bool ysfEnable    = (data[1U] & 0x04U) == 0x04U;
#endif
#if defined(MODE_P25)
  bool p25Enable    = (data[1U] & 0x08U) == 0x08U;
#endif
#if defined(MODE_NXDN)
  bool nxdnEnable   = (data[1U] & 0x10U) == 0x10U;
#endif
#if defined(MODE_FM)
  bool fmEnable     = (data[1U] & 0x20U) == 0x20U;
#endif
#if defined(MODE_POCSAG)
  bool pocsagEnable = (data[2U] & 0x01U) == 0x01U;
#endif

  uint8_t txDelay = data[3U];
  if (txDelay > 50U)
    return 4U;

  MMDVM_STATE modemState = MMDVM_STATE(data[4U]);

  if (modemState != MMDVM_STATE::IDLE && modemState != MMDVM_STATE::DSTAR && modemState != MMDVM_STATE::DMR && modemState != MMDVM_STATE::YSF && modemState != MMDVM_STATE::P25 && modemState != MMDVM_STATE::NXDN && modemState != MMDVM_STATE::POCSAG && modemState != MMDVM_STATE::FM)
    return 4U;

#if defined(MODE_DSTAR)
  if (modemState == MMDVM_STATE::DSTAR && !dstarEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::DSTAR)
    return 4U;
#endif

#if defined(MODE_DMR)
  if (modemState == MMDVM_STATE::DMR && !dmrEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::DMR)
    return 4U;
#endif

#if defined(MODE_YSF)
  if (modemState == MMDVM_STATE::YSF && !ysfEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::YSF)
    return 4U;
#endif

#if defined(MODE_P25)
  if (modemState == MMDVM_STATE::P25 && !p25Enable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::P25)
    return 4U;
#endif

#if defined(MODE_NXDN)
  if (modemState == MMDVM_STATE::NXDN && !nxdnEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::NXDN)
    return 4U;
#endif

#if defined(MODE_POCSAG)
  if (modemState == MMDVM_STATE::POCSAG && !pocsagEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::POCSAG)
    return 4U;
#endif

#if defined(MODE_FM)
  if (modemState == MMDVM_STATE::FM && !fmEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::FM)
    return 4U;
#endif

  uint8_t fmTXLevel     = data[16U];

#if defined(MODE_YSF)
  uint8_t ysfTXHang     = data[20U];
#endif
#if defined(MODE_P25)
  uint8_t p25TXHang     = data[21U];
#endif
#if defined(MODE_NXDN)
  uint8_t nxdnTXHang    = data[22U];
#endif

#if defined(MODE_DMR)
  uint8_t colorCode = data[26U];
  if (colorCode > 15U)
    return 4U;

  uint8_t dmrDelay = data[27U];
  bool    trunking = (data[25U] & 0x80U) == 0x80U;
#endif

  setMode(modemState);

  m_duplex       = !simplex;

#if defined(MODE_DSTAR)
  m_dstarEnable  = dstarEnable;
  dstarTX.setTXDelay(txDelay);
#endif
#if defined(MODE_DMR)
  m_dmrEnable    = dmrEnable;
  dmrDMOTX.setTXDelay(txDelay);

  dmrTX.setTrunking(trunking);
  dmrTX.setColorCode(colorCode);
  dmrRX.setColorCode(colorCode);
  dmrRX.setDelay(dmrDelay);
  dmrDMORX.setColorCode(colorCode);
  dmrIdleRX.setColorCode(colorCode);
#endif
#if defined(MODE_YSF)
  m_ysfEnable    = ysfEnable;
  ysfTX.setTXDelay(txDelay);
  ysfTX.setParams(ysfLoDev, ysfTXHang);
#endif
#if defined(MODE_P25)
  m_p25Enable    = p25Enable;
  p25TX.setTXDelay(txDelay);
  p25TX.setParams(p25TXHang);
#endif
#if defined(MODE_NXDN)
  m_nxdnEnable   = nxdnEnable;
  nxdnTX.setTXDelay(txDelay);
  nxdnTX.setParams(nxdnTXHang);
#endif
#if defined(MODE_POCSAG)
  m_pocsagEnable = pocsagEnable;
  pocsagTX.setTXDelay(txDelay);
#endif
#if defined(MODE_FM)
  m_fmEnable     = fmEnable;
  fm.setTXLevel(fmTXLevel);
#endif

  return io.setParameters();
}

#if defined(MODE_FM)
uint8_t CSerialPort::setFMParams1(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 8U)
    return 4U;

  uint8_t  speed     = data[0U];;
  uint16_t frequency = data[1U] * 10U;
  uint8_t  time      = data[2U];
  uint8_t  holdoff   = data[3U];
  uint8_t  highLevel = data[4U];
  uint8_t  lowLevel  = data[5U];

  bool callAtStart = (data[6U] & 0x01U) == 0x01U;
  bool callAtEnd   = (data[6U] & 0x02U) == 0x02U;
  bool callAtLatch = (data[6U] & 0x04U) == 0x04U;

  char callsign[50U];
  uint8_t n = 0U;
  for (uint8_t i = 7U; i < length; i++, n++)
    callsign[n] = data[i];
  callsign[n] = '\0';

  return fm.setCallsign(callsign, speed, frequency, time, holdoff, highLevel, lowLevel, callAtStart, callAtEnd, callAtLatch);
}

uint8_t CSerialPort::setFMParams2(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 6U)
    return 4U;

  uint8_t  speed     = data[0U];
  uint16_t frequency = data[1U] * 10U;
  uint8_t  minTime   = data[2U];
  uint16_t delay     = data[3U] * 10U;
  uint8_t  level     = data[4U];

  char ack[50U];
  uint8_t n = 0U;
  for (uint8_t i = 5U; i < length; i++, n++)
    ack[n] = data[i];
  ack[n] = '\0';

  return fm.setAck(ack, speed, frequency, minTime, delay, level);
}

uint8_t CSerialPort::setFMParams3(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 14U)
    return 4U;

  uint16_t timeout        = data[0U] * 5U;
  uint8_t  timeoutLevel   = data[1U];

  uint8_t  ctcssFrequency     = data[2U];
  uint8_t  ctcssHighThreshold = data[3U];
  uint8_t  ctcssLowThreshold  = data[4U];
  uint8_t  ctcssLevel         = data[5U];

  uint8_t  kerchunkTime   = data[6U];
  uint8_t  hangTime       = data[7U];

  uint8_t  accessMode     = data[8U] & 0x0FU;
  bool     linkMode       = (data[8U] & 0x20U) == 0x20U;
  bool     cosInvert      = (data[8U] & 0x80U) == 0x80U;

  uint8_t  rfAudioBoost   = data[9U];
  uint8_t  maxDev         = data[10U];
  uint8_t  rxLevel        = data[11U];

  uint8_t  squelchHighThreshold = data[12U];
  uint8_t  squelchLowThreshold  = data[13U];

  return fm.setMisc(timeout, timeoutLevel, ctcssFrequency, ctcssHighThreshold, ctcssLowThreshold, ctcssLevel, kerchunkTime, hangTime, accessMode, linkMode, cosInvert, squelchHighThreshold, squelchLowThreshold, rfAudioBoost, maxDev, rxLevel);
}

uint8_t CSerialPort::setFMParams4(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 4U)
    return 4U;

  uint8_t  audioBoost = data[0U];
  uint8_t  speed      = data[1U];
  uint16_t frequency  = data[2U] * 10U;
  uint8_t  level      = data[3U];

  char ack[50U];
  uint8_t n = 0U;
  for (uint8_t i = 4U; i < length; i++, n++)
    ack[n] = data[i];
  ack[n] = '\0';

  return fm.setExt(ack, audioBoost, speed, frequency, level);
}
#endif

uint8_t CSerialPort::setMode(const uint8_t* data, uint16_t length)
{
  assert(data != nullptr);

  if (length < 1U)
    return 4U;

  MMDVM_STATE modemState = MMDVM_STATE(data[0U]);

  if (modemState == m_modemState)
    return 0U;

  if (modemState != MMDVM_STATE::IDLE && modemState != MMDVM_STATE::DSTAR && modemState != MMDVM_STATE::DMR && modemState != MMDVM_STATE::YSF && modemState != MMDVM_STATE::P25 && modemState != MMDVM_STATE::NXDN && modemState != MMDVM_STATE::POCSAG && modemState != MMDVM_STATE::FM)
    return 4U;

#if defined(MODE_DSTAR)
  if (modemState == MMDVM_STATE::DSTAR && !m_dstarEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::DSTAR)
    return 4U;
#endif

#if defined(MODE_DMR)
  if (modemState == MMDVM_STATE::DMR && !m_dmrEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::DMR)
    return 4U;
#endif

#if defined(MODE_YSF)
  if (modemState == MMDVM_STATE::YSF && !m_ysfEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::YSF)
    return 4U;
#endif

#if defined(MODE_P25)
  if (modemState == MMDVM_STATE::P25 && !m_p25Enable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::P25)
    return 4U;
#endif

#if defined(MODE_NXDN)
  if (modemState == MMDVM_STATE::NXDN && !m_nxdnEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::NXDN)
    return 4U;
#endif

#if defined(MODE_POCSAG)
  if (modemState == MMDVM_STATE::POCSAG && !m_pocsagEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::POCSAG)
    return 4U;
#endif

#if defined(MODE_FM)
  if (modemState == MMDVM_STATE::FM && !m_fmEnable)
    return 4U;
#else
  if (modemState == MMDVM_STATE::FM)
    return 4U;
#endif

  setMode(modemState);

  return 0U;
}

void CSerialPort::setMode(MMDVM_STATE modemState)
{
  switch (modemState) {
    case MMDVM_STATE::DSTAR:
      LogMessage("Mode set to D-Star");
      break;
    case MMDVM_STATE::DMR:
      LogMessage("Mode set to DMR");
      break;
    case MMDVM_STATE::YSF:
      LogMessage("Mode set to System Fusion");
      break;
    case MMDVM_STATE::P25:
      LogMessage("Mode set to P25");
      break;
    case MMDVM_STATE::NXDN:
      LogMessage("Mode set to NXDN");
      break;
    case MMDVM_STATE::POCSAG:
      LogMessage("Mode set to POCSAG");
      break;
    case MMDVM_STATE::FM:
      LogMessage("Mode set to FM");
      break;
    default:        // MMDVM_STATE::IDLE
      LogMessage("Mode set to Idle");
      break;
  }

#if defined(MODE_DSTAR)
  if (modemState != MMDVM_STATE::DSTAR)
    dstarRX.reset();
#endif

#if defined(MODE_DMR)
  if (modemState != MMDVM_STATE::DMR) {
    dmrIdleRX.reset();
    dmrDMORX.reset();
    dmrRX.reset();
  }
#endif

#if defined(MODE_YSF)
  if (modemState != MMDVM_STATE::YSF)
    ysfRX.reset();
#endif

#if defined(MODE_P25)
  if (modemState != MMDVM_STATE::P25)
    p25RX.reset();
#endif

#if defined(MODE_NXDN)
  if (modemState != MMDVM_STATE::NXDN)
    nxdnRX.reset();
#endif

#if defined(MODE_FM)
  if (modemState != MMDVM_STATE::FM)
    fm.reset();
#endif

  cwIdTX.reset();

  io.setMode(modemState);
}

bool CSerialPort::start(const std::string& myAddress, unsigned short myPort, const std::string& hostAddress, unsigned short hostPort, bool debug)
{
    CSocket::startup();

    m_trace = debug;

    return m_socket.open(myAddress, myPort, hostAddress, hostPort);
}

void CSerialPort::stop()
{
}

void CSerialPort::process()
{
  while (m_socket.available()) {
    uint8_t c = m_socket.read();

    if (m_ptr == 0U) {
      if (c == MMDVM_FRAME_START) {
        // Handle the frame start correctly
        m_buffer[0U] = c;
        m_ptr = 1U;
        m_len = 0U;
      } else {
        m_ptr = 0U;
        m_len = 0U;
      }
    } else if (m_ptr == 1U) {
      // Handle the frame length, 1/2
      m_len = m_buffer[m_ptr] = c;
      m_ptr = 2U;
    } else if (m_ptr == 2U) {
      // Handle the frame length, 2/2
      m_buffer[m_ptr] = c;
      m_ptr = 3U;

      if (m_len == 0U)
        m_len = c + 255U;

      // The full packet has been received, process it
      if (m_ptr == m_len)
          processMessage(m_buffer[2U], m_buffer + 3U, m_len - 3U);
    } else {
      // Any other bytes are added to the buffer
      m_buffer[m_ptr] = c;
      m_ptr++;

      // The full packet has been received, process it
      if (m_ptr == m_len) {
          if (m_trace)
              dump("RX Host", m_buffer, m_len);

          if (m_len > 255U)
            processMessage(m_buffer[3U], m_buffer + 4U, m_len - 4U);
          else
            processMessage(m_buffer[2U], m_buffer + 3U, m_len - 3U);
      }
    }
  }
}

void CSerialPort::processMessage(uint8_t type, const uint8_t* buffer, uint16_t length)
{
    assert(buffer != nullptr);

    uint8_t err = 2U;

  switch (type) {
    case MMDVM_GET_STATUS:
      getStatus();
      break;

    case MMDVM_GET_VERSION:
      getVersion();
      break;

    case MMDVM_SET_CONFIG:
      err = setConfig(buffer, length);
      if (err == 0U)
        sendACK(type);
      else
        sendNAK(type, err);
      break;

    case MMDVM_SET_MODE:
      err = setMode(buffer, length);
      if (err == 0U)
        sendACK(type);
      else
        sendNAK(type, err);
      break;

    case MMDVM_SET_FREQ:
      err = setFrequency(buffer, length);
      if (err == 0U)
          sendACK(type);
      else
          sendNAK(type, err);
      break;

#if defined(MODE_FM)
    case MMDVM_FM_PARAMS1:
      err = setFMParams1(buffer, length);
      if (err == 0U) {
        sendACK(type);
      } else {
        LogWarning("Received invalid FM params 1, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_FM_PARAMS2:
      err = setFMParams2(buffer, length);
      if (err == 0U) {
        sendACK(type);
      } else {
        LogWarning("Received invalid FM params 2, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_FM_PARAMS3:
      err = setFMParams3(buffer, length);
      if (err == 0U) {
        sendACK(type);
      } else {
        LogWarning("Received invalid FM params 3, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_FM_PARAMS4:
      err = setFMParams4(buffer, length);
      if (err == 0U) {
        sendACK(type);
      } else {
        LogWarning("Received invalid FM params 4, err=%u", err);
        sendNAK(type, err);
      }
      break;
#else
    case MMDVM_FM_PARAMS1:
    case MMDVM_FM_PARAMS2:
    case MMDVM_FM_PARAMS3:
    case MMDVM_FM_PARAMS4:
      sendACK(type);
      break;
#endif

    case MMDVM_SEND_CWID:
      err = 5U;
      if (m_modemState == MMDVM_STATE::IDLE)
        err = cwIdTX.write(buffer, length);
      if (err != 0U) {
        LogWarning("Invalid CW Id data, err=%u", err);
        sendNAK(type, err);
      }
      break;

#if defined(MODE_DSTAR)
    case MMDVM_DSTAR_HEADER:
      if (m_dstarEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::DSTAR)
          err = dstarTX.writeHeader(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::DSTAR);
      } else {
        LogWarning("Received invalid D-Star header, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DSTAR_DATA:
      if (m_dstarEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::DSTAR)
          err = dstarTX.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::DSTAR);
      } else {
        LogWarning("Received invalid D-Star data, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DSTAR_EOT:
      if (m_dstarEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::DSTAR)
          err = dstarTX.writeEOT();
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::DSTAR);
      } else {
        LogWarning("Received invalid D-Star EOT, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

#if defined(MODE_DMR)
    case MMDVM_DMR_DATA1:
      if (m_dmrEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::DMR) {
          if (m_duplex)
            err = dmrTX.writeData1(buffer, length);
        }
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::DMR);
      } else {
        LogWarning("Received invalid DMR data, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DMR_DATA2:
      if (m_dmrEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::DMR) {
          if (m_duplex)
            err = dmrTX.writeData2(buffer, length);
          else
            err = dmrDMOTX.writeData(buffer, length);
        }
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::DMR);
      } else {
        LogWarning("Received invalid DMR data, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DMR_START:
      if (m_dmrEnable) {
        err = 4U;
        if (length == 1U) {
          if (buffer[0U] == 0x01U && m_modemState == MMDVM_STATE::DMR) {
            if (!m_tx)
              dmrTX.setStart(true);
            err = 0U;
          } else if (buffer[0U] == 0x00U && m_modemState == MMDVM_STATE::DMR) {
            if (m_tx)
              dmrTX.setStart(false);
            err = 0U;
          }
        }
      }
      if (err != 0U) {
        LogWarning("Received invalid DMR start, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DMR_SHORTLC:
      if (m_dmrEnable)
        err = dmrTX.writeShortLC(buffer, length);
      if (err != 0U) {
        LogWarning("Received invalid DMR Short LC, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DMR_ALOHA:
      if (m_dmrEnable)
        err = dmrTX.writeAloha(buffer, length);
      if (err != 0U) {
        LogWarning("Received invalid DMR ALOHA, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_DMR_ABORT:
      if (m_dmrEnable)
        err = dmrTX.writeAbort(buffer, length);
      if (err != 0U) {
        LogWarning("Received invalid DMR Abort, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

#if defined(MODE_YSF)
    case MMDVM_YSF_DATA:
      if (m_ysfEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::YSF)
          err = ysfTX.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::YSF);
      } else {
        LogWarning("Received invalid System Fusion data, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

#if defined(MODE_P25)
    case MMDVM_P25_HDR:
      if (m_p25Enable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::P25)
          err = p25TX.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::P25);
      } else {
        LogWarning("Received invalid P25 header, err=%u", err);
        sendNAK(type, err);
      }
      break;

    case MMDVM_P25_LDU:
      if (m_p25Enable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::P25)
          err = p25TX.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::P25);
      } else {
        LogWarning("Received invalid P25 LDU, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

#if defined(MODE_NXDN)
    case MMDVM_NXDN_DATA:
      if (m_nxdnEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::NXDN)
          err = nxdnTX.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::NXDN);
      } else {
        LogWarning("Received invalid NXDN data, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

#if defined(MODE_POCSAG)
    case MMDVM_POCSAG_DATA:
      if (m_pocsagEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::POCSAG)
          err = pocsagTX.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::POCSAG);
      } else {
        LogWarning("Received invalid POCSAG data, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

#if defined(MODE_FM)
    case MMDVM_FM_DATA:
      if (m_fmEnable) {
        if (m_modemState == MMDVM_STATE::IDLE || m_modemState == MMDVM_STATE::FM)
          err = fm.writeData(buffer, length);
      }
      if (err == 0U) {
        if (m_modemState == MMDVM_STATE::IDLE)
          setMode(MMDVM_STATE::FM);
      } else {
        LogWarning("Received invalid FM data, err=%u", err);
        sendNAK(type, err);
      }
      break;
#endif

    case MMDVM_TRANSPARENT:
    case MMDVM_QSO_INFO:
      // Do nothing on the MMDVM.
      break;

    default:
      // Handle this, send a NAK back
      sendNAK(type, 1U);
      break;
  }

  m_ptr = 0U;
  m_len = 0U;
}

#if defined(MODE_DSTAR)
void CSerialPort::writeDStarHeader(const uint8_t* header, uint8_t length)
{
    assert(header != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::DSTAR && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[50U];
  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DSTAR_HEADER;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = header[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write D-Star Header", reply, count);
}

void CSerialPort::writeDStarData(const uint8_t* data, uint8_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::DSTAR && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[20U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DSTAR_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write D-Star Data", reply, count);
}

void CSerialPort::writeDStarLost()
{
  if (m_modemState != MMDVM_STATE::DSTAR && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_DSTAR_LOST;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write D-Star Lost", reply, 3U);
}

void CSerialPort::writeDStarEOT()
{
  if (m_modemState != MMDVM_STATE::DSTAR && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_DSTAR_EOT;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write D-Star EOT", reply, 3U);
}
#endif

#if defined(MODE_DMR)
void CSerialPort::writeDMRData(bool slot, const uint8_t* data, uint8_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::DMR && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_dmrEnable)
    return;

  uint8_t reply[40U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = slot ? MMDVM_DMR_DATA2 : MMDVM_DMR_DATA1;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write DMR Data", reply, count);
}

void CSerialPort::writeDMRLost(bool slot)
{
  if (m_modemState != MMDVM_STATE::DMR && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_dmrEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = slot ? MMDVM_DMR_LOST2 : MMDVM_DMR_LOST1;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write DMR Lost", reply, 3U);
}
#endif

#if defined(MODE_YSF)
void CSerialPort::writeYSFData(const uint8_t* data, uint8_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::YSF && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_ysfEnable)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_YSF_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write YSF Data", reply, count);
}

void CSerialPort::writeYSFLost()
{
  if (m_modemState != MMDVM_STATE::YSF && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_ysfEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_YSF_LOST;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write YSF Lost", reply, 3U);
}
#endif

#if defined(MODE_P25)
void CSerialPort::writeP25Hdr(const uint8_t* data, uint8_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::P25 && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[120U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_P25_HDR;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write P25 Header", reply, count);
}

void CSerialPort::writeP25Ldu(const uint8_t* data, uint8_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::P25 && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[250U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_P25_LDU;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write P25 LDU", reply, count);
}

void CSerialPort::writeP25Lost()
{
  if (m_modemState != MMDVM_STATE::P25 && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_P25_LOST;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write P25 Lost", reply, 3U);
}
#endif

#if defined(MODE_NXDN)
void CSerialPort::writeNXDNData(const uint8_t* data, uint8_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::NXDN && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_nxdnEnable)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_NXDN_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_socket.write(reply, count);

  if (m_trace)
      dump("Write NXDN Data", reply, count);
}

void CSerialPort::writeNXDNLost()
{
  if (m_modemState != MMDVM_STATE::NXDN && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_nxdnEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_NXDN_LOST;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write NXDN Lost", reply, 3U);
}
#endif

#if defined(MODE_FM)
void CSerialPort::writeFMData(const uint8_t* data, uint16_t length)
{
    assert(data != nullptr);
    assert(length > 0U);

    if (m_modemState != MMDVM_STATE::FM && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_fmEnable)
    return;

  uint8_t reply[512U];

  reply[0U] = MMDVM_FRAME_START;

  if (length > 252U) {
    reply[1U] = 0U;
    reply[2U] = (length + 4U) - 255U;
    reply[3U] = MMDVM_FM_DATA;

    for (uint16_t i = 0U; i < length; i++)
      reply[i + 4U] = data[i];

    m_socket.write(reply, length + 4U);

    if (m_trace)
        dump("Write FM Data", reply, length + 4U);
  } else {
    reply[1U] = length + 3U;
    reply[2U] = MMDVM_FM_DATA;

    for (uint16_t i = 0U; i < length; i++)
      reply[i + 3U] = data[i];

    m_socket.write(reply, length + 3U);

    if (m_trace)
        dump("Write FM Data", reply, length + 3U);
  }
}

void CSerialPort::writeFMStatus(uint8_t status)
{
  if (m_modemState != MMDVM_STATE::FM && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_fmEnable)
    return;

  uint8_t reply[10U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 4U;
  reply[2U] = MMDVM_FM_STATUS;
  reply[3U] = status;

  m_socket.write(reply, 4U);

  if (m_trace)
      dump("Write FM Status", reply, 4U);
}

void CSerialPort::writeFMRSSI(uint16_t rssi)
{
  if (m_modemState != MMDVM_STATE::FM && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_fmEnable)
    return;

  uint8_t reply[10U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 5U;
  reply[2U] = MMDVM_FM_RSSI;
  reply[3U] = (rssi >> 8) & 0xFFU;
  reply[4U] = (rssi >> 0) & 0xFFU;

  m_socket.write(reply, 5U);

  if (m_trace)
      dump("Write FM RSSI", reply, 5U);
}

void CSerialPort::writeFMEOT()
{
  if (m_modemState != MMDVM_STATE::FM && m_modemState != MMDVM_STATE::IDLE)
    return;

  if (!m_fmEnable)
    return;

  uint8_t reply[10U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_FM_EOT;

  m_socket.write(reply, 3U);

  if (m_trace)
      dump("Write FM EOT", reply, 3U);
}
#endif

void CSerialPort::dump(const char* text, const uint8_t* data, uint16_t length) const
{
    LogDebug("%s:", text);

    char line[100U];

    unsigned int offset = 0U;

    while (length > 0U) {
        ::sprintf(line, "%04X:  ", offset);

        unsigned int bytes = (length > 16U) ? 16U : length;

        for (unsigned i = 0U; i < bytes; i++)
            ::sprintf(line + ::strlen(line), "%02X ", data[offset + i]);

        for (unsigned int i = bytes; i < 16U; i++)
            ::strcat(line, "   ");

        ::strcat(line, "   *");

        for (unsigned i = 0U; i < bytes; i++) {
            uint8_t c = data[offset + i];

            if (::isprint(c))
                ::sprintf(line + ::strlen(line), "%c", c);
            else
                ::strcat(line, ".");
        }

        ::strcat(line, "*");

        LogDebug("%s", line);

        offset += 16U;

        if (length >= 16U)
            length -= 16U;
        else
            length = 0U;
    }
}
