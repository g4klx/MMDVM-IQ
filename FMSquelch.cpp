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

#include "Globals.h"
#include "FMSquelch.h"
#include "Log.h"

const uint8_t MAX_COUNT = 4U;


CFMSquelch::CFMSquelch() :
m_highThreshold(0U),
m_lowThreshold(0U),
m_count(0U),
m_state(false),
m_invert(false),
m_configured(false)
{
}

CFMSquelch::~CFMSquelch()
{
}

void CFMSquelch::setParams(uint8_t highThreshold, uint8_t lowThreshold, bool invert)
{
	m_highThreshold = highThreshold;
	m_lowThreshold  = lowThreshold;
	m_invert = invert;

	m_count = 0U;
	m_state = false;
	m_configured = true;

	LogDebug("FM: squelch parameters high/low/invert: %u/%u/%s",
		m_highThreshold,
		m_lowThreshold,
		m_invert ? "true" : "false");
}

bool CFMSquelch::process(uint16_t rssi)
{
	if (!m_configured)
		return false;
	const bool close = m_invert ?
		rssi >= m_highThreshold :
		rssi <= m_lowThreshold;

	const bool open = m_invert ?
		rssi <= m_lowThreshold :
		rssi >= m_highThreshold;

	if (m_state) {
		if (close) {
			if (m_count > 0U)
				m_count--;

			if (m_count == 0U) {
				LogDebug("FM: squelch closed, RSSI: %u", rssi);
				m_state = false;
			}
		} else {
			m_count = MAX_COUNT;
		}
	} else {
		if (open) {
			m_count++;

			if (m_count >= MAX_COUNT) {
				LogDebug("FM: squelch open, RSSI: %u", rssi);
				m_state = true;
			}
		} else {
			m_count = 0U;
		}
	}

	return m_state;
}

void CFMSquelch::reset()
{
	m_count = 0U;
	m_state = false;
}

#endif
