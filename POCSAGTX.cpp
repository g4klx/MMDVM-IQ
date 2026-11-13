/*
 *   Copyright (C) 2009-2018,2020,2026 by Jonathan Naylor G4KLX
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

#if defined(MODE_POCSAG)

#include "Globals.h"
#include "POCSAGTX.h"

const uint16_t POCSAG_FRAME_LENGTH_BYTES = 17U * sizeof(uint32_t);

const uint16_t POCSAG_PREAMBLE_LENGTH_BYTES = 18U * sizeof(uint32_t);

const uint16_t POCSAG_RADIO_SYMBOL_LENGTH = 20U;

const uint16_t POCSAG_MULTI_DATA_LENGTH = 20U;

const q15_t POCSAG_LEVEL1[] = { 3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875,  3875};
const q15_t POCSAG_LEVEL0[] = {-3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875, -3875};

static q15_t SHAPING_FILTER[] = {5461, 5461, 5461, 5461, 5461, 5461};
const uint16_t SHAPING_FILTER_LEN = 6U;

const uint8_t POCSAG_SYNC = 0xAAU;

CPOCSAGTX::CPOCSAGTX() :
m_inBuffer(4000U, "POCSAG Input Buffer"),
m_outBuffer(500U, "POCSAG Output Buffer"),
m_modFilter(),
m_modState(),
m_poBuffer(),
m_poLen(0U),
m_poPtr(0U),
m_txDelay(POCSAG_PREAMBLE_LENGTH_BYTES)
{
  ::memset(m_modState, 0x00U, 170U * sizeof(q15_t));

  m_modFilter.numTaps = SHAPING_FILTER_LEN;
  m_modFilter.pState  = m_modState;
  m_modFilter.pCoeffs = SHAPING_FILTER;
}

CPOCSAGTX::~CPOCSAGTX()
{
}

void CPOCSAGTX::process()
{
  if (m_inBuffer.isEmpty() && m_poLen == 0U)
    return;

  if (m_poLen == 0U) {
    if (!m_tx) {
      for (uint16_t i = 0U; i < m_txDelay; i++)
        m_poBuffer[m_poLen++] = POCSAG_SYNC;
    } else {
      for (uint8_t i = 0U; i < POCSAG_FRAME_LENGTH_BYTES; i++)
        m_inBuffer.getData(m_poBuffer[m_poLen++]);
    }

    m_poPtr = 0U;
  }

  if (m_poLen > 0U) {
    unsigned int space = m_outBuffer.freeSpace();
    
    while (space > (8U * POCSAG_RADIO_SYMBOL_LENGTH)) {
      uint8_t c = m_poBuffer[m_poPtr++];
      writeByte(c);

      space -= 8U * POCSAG_RADIO_SYMBOL_LENGTH;
      
      if (m_poPtr >= m_poLen) {
        m_poPtr = 0U;
        m_poLen = 0U;
        break;
      }
    }
  }
  
  // Feed the POCSAG data out to the SDR in blocks of POCSAG_MULTI_DATA_LENGTH samples.
  uint16_t ioSpace     = io.getSpace();
  unsigned int outData = m_outBuffer.dataSize();

  while ((ioSpace >= POCSAG_MULTI_DATA_LENGTH) && (outData >= POCSAG_MULTI_DATA_LENGTH)) {
    q15_t buffer[POCSAG_MULTI_DATA_LENGTH];
    m_outBuffer.getData(buffer, POCSAG_MULTI_DATA_LENGTH);

    io.write(MMDVM_STATE::POCSAG, buffer, POCSAG_MULTI_DATA_LENGTH);

    ioSpace -= POCSAG_MULTI_DATA_LENGTH;
    outData -= POCSAG_MULTI_DATA_LENGTH;
  }
}

bool CPOCSAGTX::busy()
{
  if (m_poLen > 0U || m_inBuffer.hasData())
    return true;
  else
    return false;
}

uint8_t CPOCSAGTX::writeData(const uint8_t* data, uint16_t length)
{
  if (length != POCSAG_FRAME_LENGTH_BYTES)
    return 4U;

  uint16_t space = m_inBuffer.freeSpace();
  if (space < POCSAG_FRAME_LENGTH_BYTES)
    return 5U;

  m_inBuffer.addData(data, POCSAG_FRAME_LENGTH_BYTES);

  return 0U;
}

void CPOCSAGTX::writeByte(uint8_t c)
{
  q15_t inBuffer[POCSAG_RADIO_SYMBOL_LENGTH * 8U];
  q15_t outBuffer[POCSAG_RADIO_SYMBOL_LENGTH * 8U];

  const uint8_t MASK = 0x80U;

  uint8_t n = 0U;
  for (uint8_t i = 0U; i < 8U; i++, c <<= 1, n += POCSAG_RADIO_SYMBOL_LENGTH) {
    switch (c & MASK) {
      case 0x80U:
        ::memcpy(inBuffer + n, POCSAG_LEVEL1, POCSAG_RADIO_SYMBOL_LENGTH * sizeof(q15_t));
        break;
      default:
        ::memcpy(inBuffer + n, POCSAG_LEVEL0, POCSAG_RADIO_SYMBOL_LENGTH * sizeof(q15_t));
        break;
    }
  }

  ::arm_fir_fast_q15(&m_modFilter, inBuffer, outBuffer, POCSAG_RADIO_SYMBOL_LENGTH * 8U);

  m_outBuffer.addData(outBuffer, POCSAG_RADIO_SYMBOL_LENGTH * 8U);
}

void CPOCSAGTX::setTXDelay(uint8_t delay)
{
  m_txDelay = POCSAG_PREAMBLE_LENGTH_BYTES + (delay * 3U) / 2U;

  if (m_txDelay > 150U)
    m_txDelay = 150U;
}

uint8_t CPOCSAGTX::getSpace() const
{
  return m_inBuffer.freeSpace() / POCSAG_FRAME_LENGTH_BYTES;
}

#endif
