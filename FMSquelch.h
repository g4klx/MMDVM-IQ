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

#if !defined(FMSQUELCH_H)
#define  FMSQUELCH_H

class CFMSquelch {
public:
	CFMSquelch();
	~CFMSquelch();

	void setParams(uint8_t highThreshold, uint8_t lowThreshold, bool invert);
  
	bool process(uint16_t rssi);

	void reset();

private:
	uint16_t m_highThreshold;
	uint16_t m_lowThreshold;
	uint8_t  m_count;
	bool     m_state;
	bool	 m_invert;
	bool     m_configured;
};

#endif

#endif
