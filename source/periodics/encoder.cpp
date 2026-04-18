/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#include <periodics/encoder.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#define PI_FLOAT 3.14159265358979323846
#define MIN_HAMPEL_THRESHOLD 0.5f
#define HAMPEL_K 3.0f
#define MAX_ENCODER_ANGULAR_SPEED_DEG_PER_SEC 12000.0f
#define MAX_ENCODER_STEP_MARGIN_DEG 20.0f

namespace
{
    constexpr float c_piFloat = 3.14159265358979323846f;
    constexpr float c_wheelDiameterMm = 67.0f;
    constexpr float c_motorShaftRatio = 62.0f / 24.0f;
    constexpr float c_shaftDiffRatio = 34.0f / 11.0f;
    constexpr float c_encoderDistanceScale = 1.0f;
    constexpr float c_wheelCircumferenceMm = c_wheelDiameterMm * c_piFloat;
    constexpr float c_totalGearRatio = c_motorShaftRatio * c_shaftDiffRatio;
    constexpr uint8_t c_statusMagnetDetectedMask = 1U << 5;
    constexpr uint8_t c_statusMagnetTooWeakMask = 1U << 4;
    constexpr uint8_t c_statusMagnetTooStrongMask = 1U << 3;
    constexpr uint16_t c_confClearHysteresisAndPowerModeMask = 0xFFF0U;

    bool readRegisters(I2C& f_i2c, int f_address, char f_register, char* f_data, int f_length)
    {
        char l_register = f_register;
        if (0 != f_i2c.write(f_address, &l_register, 1, true))
        {
            return false;
        }

        return 0 == f_i2c.read(f_address, f_data, f_length, false);
    }

    bool writeRegisters(I2C& f_i2c, int f_address, char f_register, const char* f_data, int f_length)
    {
        char l_buffer[3] = {0, 0, 0};

        if ((f_length <= 0) || (f_length > 2))
        {
            return false;
        }

        l_buffer[0] = f_register;
        std::memcpy(&l_buffer[1], f_data, static_cast<size_t>(f_length));
        return 0 == f_i2c.write(f_address, l_buffer, f_length + 1, false);
    }
}

namespace periodics
{
    CEncoderKalman2D::CEncoderKalman2D()
        : angle(0.0f)
        , speed(0.0f)
        , p00(1.0f)
        , p01(0.0f)
        , p11(1.0f)
        , q_angle(0.005f)
        , q_speed(6.0f)
        , r_angle(0.02f)
    {
    }

    void CEncoderKalman2D::predict(float f_dt)
    {
        const float l_p00 = p00 + f_dt * (p01 + p01 + p11 * f_dt);
        const float l_p01 = p01 + p11 * f_dt;
        const float l_p11 = p11 + q_speed;

        angle += f_dt * speed;
        p00 = l_p00 + q_angle;
        p01 = l_p01;
        p11 = l_p11;
    }

    void CEncoderKalman2D::update(float f_measurement)
    {
        const float l_innovation = f_measurement - angle;
        const float l_s = p00 + r_angle;
        const float l_k0 = p00 / l_s;
        const float l_k1 = p01 / l_s;

        angle += l_k0 * l_innovation;
        speed += l_k1 * l_innovation;

        const float l_p00 = p00 - l_k0 * p00;
        const float l_p01 = p01 - l_k0 * p01;
        const float l_p11 = p11 - l_k1 * p01;

        p00 = l_p00;
        p01 = l_p01;
        p11 = l_p11;
    }

    CEncoder::CEncoder(
            std::chrono::milliseconds f_period,
            drivers::CSerialTxBroker& f_serialBroker,
            PinName f_sdaPin,
            PinName f_sclPin
        )
        : utils::CTask(f_period)
        , m_i2c(f_sdaPin, f_sclPin)
        , m_serialBroker(f_serialBroker)
        , m_isActive(false)
        , m_lastAngularSpeed(0.0f)
        , m_lastLinearSpeed(0.0f)
        , m_sensorConfigured(false)
        , m_magnetDetected(false)
        , m_magnetTooWeak(false)
        , m_magnetTooStrong(false)
        , m_lastAgc(0U)
        , m_lastMagnitude(0U)
        , m_diagnosticRefreshCounter(0U)
        , m_previousRawAngle(0.0f)
        , m_lastRawAngleDegrees(0.0f)
        , m_unwrapRevolutions(0)
        , m_publishAccumulator(0.0f)
        , m_missingMeasurementDuration(0.0f)
        , m_lastTimerUs(0)
        , m_totalDisplacement(0.0f)
        , m_hasDisplacementReference(false)
        , m_lastDisplacementRawAngle(0.0f)
        , m_timerStarted(false)
        , m_hampelBuffer{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
        , m_hampelIndex(0U)
        , m_hampelCount(0U)
        , m_speedHysteresis(10.0f)
        , m_reportInterval(static_cast<float>(c_defaultReportIntervalMs) * 0.001f)
        , m_timer()
        , m_kalman()
    {
        m_i2c.frequency(400000);
    }

    CEncoder::~CEncoder()
    {
    };

    void CEncoder::serialCallbackENCODERcommand(char const * a, char * b)
    {
        if (0 == strncmp(a, "diag", 4))
        {
            snprintf(
                b,
                64,
                "md=%u ml=%u mh=%u agc=%u mag=%u raw=%u",
                m_magnetDetected ? 1U : 0U,
                m_magnetTooWeak ? 1U : 0U,
                m_magnetTooStrong ? 1U : 0U,
                static_cast<unsigned int>(m_lastAgc),
                static_cast<unsigned int>(m_lastMagnitude),
                static_cast<unsigned int>(roundf(m_lastRawAngleDegrees))
            );
            return;
        }

        unsigned int l_isActivate = 0U;
        unsigned int l_reportIntervalMs = c_defaultReportIntervalMs;
        const int l_res = sscanf(a, "%u;%u", &l_isActivate, &l_reportIntervalMs);

        if (l_res == 1 || l_res == 2)
        {
            if (l_res == 2)
            {
                if (l_reportIntervalMs < c_minReportIntervalMs)
                {
                    l_reportIntervalMs = c_minReportIntervalMs;
                }
                m_reportInterval = static_cast<float>(l_reportIntervalMs) * 0.001f;
            }

            m_isActive = (l_isActivate >= 1U);
            sprintf(b, "1");
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    bool CEncoder::ensureSensorConfigured()
    {
        if (m_sensorConfigured)
        {
            return true;
        }

        char l_data[2] = {0, 0};
        if (!readRegisters(m_i2c, c_as5600Address, c_confHighRegister, l_data, 2))
        {
            return false;
        }

        const uint16_t l_conf = (static_cast<uint16_t>(static_cast<uint8_t>(l_data[0])) << 8)
                              | static_cast<uint8_t>(l_data[1]);
        const uint16_t l_updatedConf = l_conf & c_confClearHysteresisAndPowerModeMask;

        if (l_updatedConf != l_conf)
        {
            const char l_writeData[2] = {
                static_cast<char>((l_updatedConf >> 8) & 0xFFU),
                static_cast<char>(l_updatedConf & 0xFFU)
            };

            if (!writeRegisters(m_i2c, c_as5600Address, c_confHighRegister, l_writeData, 2))
            {
                return false;
            }
        }

        m_sensorConfigured = true;
        return true;
    }

    void CEncoder::refreshDiagnostics()
    {
        char l_agcData[1] = {0};
        if (readRegisters(m_i2c, c_as5600Address, c_agcRegister, l_agcData, 1))
        {
            m_lastAgc = static_cast<uint8_t>(l_agcData[0]);
        }

        char l_magnitudeData[2] = {0, 0};
        if (readRegisters(m_i2c, c_as5600Address, c_magnitudeHighRegister, l_magnitudeData, 2))
        {
            m_lastMagnitude = ((static_cast<uint16_t>(static_cast<uint8_t>(l_magnitudeData[0])) & 0x0FU) << 8)
                            | static_cast<uint8_t>(l_magnitudeData[1]);
        }
    }

    bool CEncoder::readRawAngleDegrees(float& f_angleDegrees)
    {
        (void)ensureSensorConfigured();

        char l_data[3] = {0, 0, 0};
        if (!readRegisters(m_i2c, c_as5600Address, c_statusRegister, l_data, 3))
        {
            return false;
        }

        const uint8_t l_status = static_cast<uint8_t>(l_data[0]);
        m_magnetDetected = (l_status & c_statusMagnetDetectedMask) != 0U;
        m_magnetTooWeak = (l_status & c_statusMagnetTooWeakMask) != 0U;
        m_magnetTooStrong = (l_status & c_statusMagnetTooStrongMask) != 0U;

        if (m_diagnosticRefreshCounter == 0U)
        {
            refreshDiagnostics();
            m_diagnosticRefreshCounter = c_diagnosticRefreshDivider;
        }
        else
        {
            --m_diagnosticRefreshCounter;
        }

        if (!m_magnetDetected)
        {
            return false;
        }

        const uint16_t l_rawAngle = ((static_cast<uint16_t>(static_cast<uint8_t>(l_data[1])) & 0x0FU) << 8)
                                  | static_cast<uint8_t>(l_data[2]);

        f_angleDegrees = static_cast<float>(l_rawAngle) * 360.0f / 4096.0f;
        m_lastRawAngleDegrees = f_angleDegrees;
        return true;
    }

    float CEncoder::getRawAngleDegrees()
    {
        float l_rawAngleDegrees = m_lastRawAngleDegrees;
        (void)readRawAngleDegrees(l_rawAngleDegrees);
        return l_rawAngleDegrees;
    }

    float CEncoder::readAngularSpeed()
    {
        if (!m_timerStarted)
        {
            return readAngularSpeedKalman();
        }

        return m_lastAngularSpeed;
    }

    float CEncoder::getTotalDisplacementDegrees()
    {
        if (!m_timerStarted)
        {
            (void)readAngularSpeedKalman();
        }

        return m_totalDisplacement;
    }

    void CEncoder::resetTravelDistance()
    {
        m_totalDisplacement = 0.0f;
        if (m_timerStarted)
        {
            m_lastDisplacementRawAngle =
                m_lastRawAngleDegrees +
                (360.0f * static_cast<float>(m_unwrapRevolutions));
            m_hasDisplacementReference = true;
        }
        else
        {
            m_lastDisplacementRawAngle = 0.0f;
            m_hasDisplacementReference = false;
        }
    }

    float CEncoder::getTravelDistanceMm()
    {
        return convertAngularToLinear(getTotalDisplacementDegrees());
    }

    float CEncoder::getLinearSpeed()
    {
        if (!m_timerStarted)
        {
            (void)readAngularSpeedKalman();
        }

        return m_lastLinearSpeed;
    }

    float CEncoder::readAngularSpeedKalman()
    {
        if (!m_timerStarted)
        {
            m_timer.start();
            m_timerStarted = true;
        }

        const auto l_nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            m_timer.elapsed_time()
        ).count();
        if (m_lastTimerUs == 0)
        {
            m_lastTimerUs = l_nowUs;
            float l_initialRawAngle = m_lastRawAngleDegrees;
            (void)readRawAngleDegrees(l_initialRawAngle);
            m_previousRawAngle = l_initialRawAngle;
            if (!m_hasDisplacementReference)
            {
                m_lastDisplacementRawAngle = l_initialRawAngle;
                m_hasDisplacementReference = true;
            }
            return 0.0f;
        }

        const float l_dt = static_cast<float>(l_nowUs - m_lastTimerUs) * 1e-6f;
        m_lastTimerUs = l_nowUs;

        if (l_dt <= 0.0f)
        {
            return m_lastAngularSpeed;
        }

        float l_rawAngleDegrees = m_lastRawAngleDegrees;
        const bool l_hasMeasurement = readRawAngleDegrees(l_rawAngleDegrees);
        if (!l_hasMeasurement)
        {
            m_missingMeasurementDuration += l_dt;
            if (m_missingMeasurementDuration >= 0.12f)
            {
                m_lastAngularSpeed = 0.0f;
                m_lastLinearSpeed = 0.0f;
                m_kalman.speed = 0.0f;
            }
            m_publishAccumulator += l_dt;
            return m_lastAngularSpeed;
        }

        m_missingMeasurementDuration = 0.0f;

        float l_deltaAngleDegrees = l_rawAngleDegrees - m_previousRawAngle;
        const float l_rawDeltaAngleDegrees = l_deltaAngleDegrees;

        if (l_deltaAngleDegrees > 180.0f)
        {
            l_deltaAngleDegrees -= 360.0f;
        }
        else if (l_deltaAngleDegrees < -180.0f)
        {
            l_deltaAngleDegrees += 360.0f;
        }

        const float l_maxAcceptedDeltaDegrees =
            (MAX_ENCODER_ANGULAR_SPEED_DEG_PER_SEC * l_dt) + MAX_ENCODER_STEP_MARGIN_DEG;
        if (fabsf(l_deltaAngleDegrees) > l_maxAcceptedDeltaDegrees)
        {
            m_publishAccumulator += l_dt;
            return m_lastAngularSpeed;
        }

        if (l_rawDeltaAngleDegrees > 180.0f)
        {
            m_unwrapRevolutions--;
        }
        else if (l_rawDeltaAngleDegrees < -180.0f)
        {
            m_unwrapRevolutions++;
        }

        m_previousRawAngle = l_rawAngleDegrees;

        const float l_measurementAngle = l_rawAngleDegrees + 360.0f * static_cast<float>(m_unwrapRevolutions);
        const float l_rawSpeedDegPerSec = l_deltaAngleDegrees / l_dt;
        if (!m_hasDisplacementReference)
        {
            m_lastDisplacementRawAngle = l_measurementAngle;
            m_hasDisplacementReference = true;
        }
        else
        {
            m_totalDisplacement += (l_measurementAngle - m_lastDisplacementRawAngle);
            m_lastDisplacementRawAngle = l_measurementAngle;
        }

        m_kalman.predict(l_dt);
        m_kalman.update(l_measurementAngle);

        float l_speedDegPerSec = applyHampel(0.5f * (m_kalman.speed + l_rawSpeedDegPerSec));
        l_speedDegPerSec = applySpeedHysteresis(l_speedDegPerSec);
        m_lastAngularSpeed = l_speedDegPerSec;
        m_lastLinearSpeed = convertAngularToLinear(l_speedDegPerSec);

        m_publishAccumulator += l_dt;

        return m_lastAngularSpeed;
    }

    float CEncoder::applyHampel(float f_newSample)
    {
        m_hampelBuffer[m_hampelIndex] = f_newSample;
        m_hampelIndex = (m_hampelIndex + 1U) % c_hampelWindow;

        if (m_hampelCount < c_hampelWindow)
        {
            ++m_hampelCount;
            return f_newSample;
        }

        float l_sorted[c_hampelWindow];
        float l_deviation[c_hampelWindow];

        for (size_t i = 0U; i < c_hampelWindow; ++i)
        {
            l_sorted[i] = m_hampelBuffer[i];
        }

        std::sort(l_sorted, l_sorted + c_hampelWindow);
        const float l_median = l_sorted[c_hampelWindow / 2U];

        for (size_t i = 0U; i < c_hampelWindow; ++i)
        {
            l_deviation[i] = fabsf(m_hampelBuffer[i] - l_median);
        }

        std::sort(l_deviation, l_deviation + c_hampelWindow);
        const float l_mad = l_deviation[c_hampelWindow / 2U];
        float l_threshold = HAMPEL_K * l_mad;

        if (l_threshold < MIN_HAMPEL_THRESHOLD)
        {
            l_threshold = MIN_HAMPEL_THRESHOLD;
        }

        if (fabsf(f_newSample - l_median) > l_threshold)
        {
            return l_median;
        }

        return f_newSample;
    }

    float CEncoder::applySpeedHysteresis(float f_speed)
    {
        if (fabsf(f_speed) <= m_speedHysteresis)
        {
            return 0.0f;
        }
        return f_speed;
    }

    float CEncoder::convertAngularToLinear(float f_angularQuantity) const
    {
        return
            c_encoderDistanceScale *
            ((f_angularQuantity * c_wheelCircumferenceMm) / (360.0f * c_totalGearRatio));
    }

    void CEncoder::publishSpeed()
    {
        char l_buffer[64];
        const int l_speedMmPerSec = static_cast<int>(roundf(m_lastLinearSpeed));

        snprintf(l_buffer, sizeof(l_buffer), "@encoder:%d;;\r\n", l_speedMmPerSec);
        m_serialBroker.publishLatest(drivers::CSerialTxBroker::TelemetryTopic::Encoder, l_buffer, strlen(l_buffer));
    }

    void CEncoder::_run()
    {
        (void)readAngularSpeedKalman();

        if (!m_isActive)
        {
            return;
        }

        if (m_publishAccumulator >= m_reportInterval)
        {
            m_publishAccumulator = 0.0f;
            publishSpeed();
        }
    }
}; // namespace periodics
