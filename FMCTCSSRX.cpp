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
#include "FMCTCSSRX.h"

const struct RX_CTCSS_TABLE {
  uint8_t frequency;
  q63_t   coeffDivTwo;
} RX_CTCSS_TABLE_DATA[] = {
  { 67U, 2147153298},
  { 69U, 2147130228},
  { 71U, 2147103212},
  { 74U, 2147076297},
  { 77U, 2147047330},
  { 79U, 2147016195},
  { 82U, 2146982775},
  { 85U, 2146946945},
  { 88U, 2146907275},
  { 91U, 2146867538},
  { 94U, 2146822298},
  { 97U, 2146785526},
  {100U, 2146747759},
  {103U, 2146695349},
  {107U, 2146637984},
  {110U, 2146578604},
  {114U, 2146513835},
  {118U, 2146445080},
  {123U, 2146370355},
  {127U, 2146291161},
  {131U, 2146205372},
  {136U, 2146112589},
  {141U, 2146014479},
  {146U, 2145910829},
  {151U, 2145796971},
  {156U, 2145676831},
  {159U, 2145604646},
  {162U, 2145547790},
  {165U, 2145468230},
  {167U, 2145409363},
  {171U, 2145324517},
  {173U, 2145261046},
  {177U, 2145170643},
  {179U, 2145102321},
  {183U, 2145006080},
  {186U, 2144932648},
  {189U, 2144830280},
  {192U, 2144748638},
  {196U, 2144639788},
  {199U, 2144555290},
  {203U, 2144436713},
  {206U, 2144346237},
  {210U, 2144217348},
  {218U, 2143983951},
  {225U, 2143735870},
  {229U, 2143622139},
  {233U, 2143469001},
  {241U, 2143182299},
  {250U, 2142874683},
  {254U, 2142733729}};

const uint8_t CTCSS_TABLE_DATA_LEN = 50U;

// 4Hz bandwidth
const uint16_t N = 24000U / 4U;

const float SAMPLE_RATE          = 24000.0F;
const float TWO_PI               = 6.28318530717958647692F;
const float Q31_SCALE            = 2147483648.0F;
const float GUARD_OFFSET         = 4.0F;
const float GUARD_DOMINANCE      = 2.0F;
const float MINIMUM_MEAN_ENERGY  = 0.00000001F;
const uint8_t VALID_WINDOWS      = 2U;
const uint8_t INVALID_WINDOWS    = 2U;

static void processGoertzel(float sample, float coeff, float& q1, float& q2)
{
  const float q0 = sample + coeff * q1 - q2;
  q2 = q1;
  q1 = q0;
}

static float getGoertzelPower(float coeff, float q1, float q2)
{
  const float power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
  return power > 0.0F ? power : 0.0F;
}

CFMCTCSSRX::CFMCTCSSRX() :
m_coeff(0.0F),
m_lowerCoeff(0.0F),
m_upperCoeff(0.0F),
m_highThreshold(0U),
m_lowThreshold(0U),
m_count(0U),
m_q1(0.0F),
m_q2(0.0F),
m_lowerQ1(0.0F),
m_lowerQ2(0.0F),
m_upperQ1(0.0F),
m_upperQ2(0.0F),
m_inputEnergy(0.0F),
m_validCount(0U),
m_invalidCount(0U),
m_state(false)
{
}

CFMCTCSSRX::~CFMCTCSSRX()
{
}

uint8_t CFMCTCSSRX::setParams(uint8_t frequency, uint8_t highThreshold, uint8_t lowThreshold)
{
  q63_t coeffDivTwo = 0;

  for (uint8_t i = 0U; i < CTCSS_TABLE_DATA_LEN; i++) {
    if (RX_CTCSS_TABLE_DATA[i].frequency == frequency) {
      coeffDivTwo = RX_CTCSS_TABLE_DATA[i].coeffDivTwo;
      break;
    }
  }

  if (coeffDivTwo == 0)
    return 4U;

  const float targetCos = static_cast<float>(coeffDivTwo) / Q31_SCALE;
  const float targetAngle = std::acos(targetCos);
  const float guardAngle = TWO_PI * GUARD_OFFSET / SAMPLE_RATE;

  m_coeff      = 2.0F * targetCos;
  m_lowerCoeff = 2.0F * std::cos(targetAngle - guardAngle);
  m_upperCoeff = 2.0F * std::cos(targetAngle + guardAngle);

  m_highThreshold = highThreshold;
  m_lowThreshold  = lowThreshold;

  reset();

  return 0U;
}

bool CFMCTCSSRX::process(q15_t sample)
{
  const float sampleFloat = static_cast<float>(sample) / 32768.0F;

  processGoertzel(sampleFloat, m_coeff,      m_q1,      m_q2);
  processGoertzel(sampleFloat, m_lowerCoeff, m_lowerQ1, m_lowerQ2);
  processGoertzel(sampleFloat, m_upperCoeff, m_upperQ1, m_upperQ2);

  m_inputEnergy += sampleFloat * sampleFloat;

  m_count++;
  if (m_count == N) {
    const float targetPower = getGoertzelPower(m_coeff, m_q1, m_q2);
    const float lowerPower  = getGoertzelPower(m_lowerCoeff, m_lowerQ1, m_lowerQ2);
    const float upperPower  = getGoertzelPower(m_upperCoeff, m_upperQ1, m_upperQ2);
    const float meanEnergy  = m_inputEnergy / static_cast<float>(N);

    float ratio = 0.0F;
    if (meanEnergy >= MINIMUM_MEAN_ENERGY)
      ratio = targetPower / (static_cast<float>(N) * m_inputEnergy);

    // Threshold values are normalized ratios in ten-thousandths.
    // For example, 150 represents 0.0150.
    const uint16_t value = static_cast<uint16_t>(std::lround(ratio * 10000.0F));
    const uint8_t threshold = m_state ? m_lowThreshold : m_highThreshold;

    const bool dominant =
      targetPower >= lowerPower * GUARD_DOMINANCE &&
      targetPower >= upperPower * GUARD_DOMINANCE;
    const bool detected = value >= threshold && dominant;

    bool previousState = m_state;

    if (detected) {
      m_invalidCount = 0U;

      if (!m_state) {
        if (m_validCount < VALID_WINDOWS)
          m_validCount++;

        if (m_validCount >= VALID_WINDOWS) {
          m_state = true;
          m_validCount = 0U;
        }
      }
    } else {
      m_validCount = 0U;

      if (m_state) {
        if (m_invalidCount < INVALID_WINDOWS)
          m_invalidCount++;

        if (m_invalidCount >= INVALID_WINDOWS) {
          m_state = false;
         m_invalidCount = 0U;
        }
      }
    }

    if (previousState != m_state)
      LogDebug("FM: CTCSS value/threshold/dominant/valid: %u/%u/%s/%s",
        value, threshold, dominant ? "true" : "false", m_state ? "true" : "false");

    m_count = 0U;
    m_q1 = 0.0F;
    m_q2 = 0.0F;
    m_lowerQ1 = 0.0F;
    m_lowerQ2 = 0.0F;
    m_upperQ1 = 0.0F;
    m_upperQ2 = 0.0F;
    m_inputEnergy = 0.0F;
  }

  return m_state;
}

void CFMCTCSSRX::reset()
{
  m_q1 = 0.0F;
  m_q2 = 0.0F;
  m_lowerQ1 = 0.0F;
  m_lowerQ2 = 0.0F;
  m_upperQ1 = 0.0F;
  m_upperQ2 = 0.0F;
  m_inputEnergy = 0.0F;
  m_validCount = 0U;
  m_invalidCount = 0U;
  m_state = false;
  m_count = 0U;
}

#endif
