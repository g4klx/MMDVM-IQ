/*
 *   Copyright (C) 2020,2026 by Jonathan Naylor G4KLX
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

#include "Config.h"

#if defined(MODE_FM)

#if !defined(FMCTCSSRX_H)
#define  FMCTCSSRX_H

class CFMCTCSSRX {
public:
  CFMCTCSSRX();
  ~CFMCTCSSRX();

  uint8_t setParams(uint8_t frequency, uint8_t highThreshold, uint8_t lowThreshold);
  
  bool process(q15_t sample);

  void reset();

private:
  float    m_coeff;
  float    m_lowerCoeff;
  float    m_upperCoeff;
  uint8_t  m_highThreshold;
  uint8_t  m_lowThreshold;
  uint16_t m_count;
  float    m_q1;
  float    m_q2;
  float    m_lowerQ1;
  float    m_lowerQ2;
  float    m_upperQ1;
  float    m_upperQ2;
  float    m_inputEnergy;
  uint8_t  m_validCount;
  uint8_t  m_invalidCount;
  bool     m_state;
};

#endif

#endif

