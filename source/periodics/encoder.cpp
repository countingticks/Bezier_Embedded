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
#define SPEED_GATE_RATIO 0.15f // reference filter
#define MIN_REFERENCE_FOR_RATIO_GATE 1.0f // reference filter
#define DEGREE_PER_MM -14.5f // not used
#define WHEEL_DIAMETER 64.0f
#define SPEED_FILTER_CUTOFF_HZ 30.0f
#define ACCELERATION_FILTER_ALPHA 0.2f
#define MIN_HAMPEL_THRESHOLD 0.5f
#define HAMPEL_K 3.0f

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

    CEncoder::CBiquad::CBiquad()
        : b0(0.0f)
        , b1(0.0f)
        , b2(0.0f)
        , a1(0.0f)
        , a2(0.0f)
        , z1(0.0f)
        , z2(0.0f)
    {
    }

    float CEncoder::CBiquad::process(float f_input)
    {
        const float l_output = b0 * f_input + b1 * z1 + b2 * z2 - a1 * z1 - a2 * z2;
        z2 = z1;
        z1 = l_output;
        return l_output;
    }

    void CEncoder::CBiquad::reset()
    {
        z1 = 0.0f;
        z2 = 0.0f;
    }

    CEncoder::CEncoder(
            std::chrono::milliseconds f_period,
            UnbufferedSerial& f_serial,
            PinName f_pwmPin
        )
        : utils::CTask(f_period)
        , m_pwm(f_pwmPin)
        , m_serial(f_serial)
        , m_isActive(false)
        , m_speedReference(0.0f)
        , m_speedReferenceHistory{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
        , m_lastAngularSpeed(0.0f)
        , m_lastAngularAcceleration(0.0f)
        , m_lastLinearSpeed(0.0f)
        , m_lastLinearAcceleration(0.0f)
        , m_previousRawAngle(0.0f)
        , m_unwrapRevolutions(0)
        , m_publishAccumulator(0.0f)
        , m_lastTimerUs(0)
        , m_totalDisplacement(0.0f)
        , m_hasDisplacementReference(false)
        , m_lastDisplacementRawAngle(0.0f)
        , m_timerStarted(false)
        , m_hampelBuffer{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
        , m_hampelIndex(0U)
        , m_hampelCount(0U)
        , m_samplingFrequency(0.0f)
        , m_angleHysteresis(1.0f)
        , m_speedHysteresis(10.0f)
        , m_lastAngle(0.0f)
        , m_reportInterval(0.02f)
        , m_timer()
        , m_kalman()
        , m_sinFilter()
        , m_cosFilter()
    {
        const float l_dt = static_cast<float>(f_period.count()) / 1000.0f;
        const float l_cutoffHz = SPEED_FILTER_CUTOFF_HZ;
        const float l_k = tanf(PI_FLOAT * l_cutoffHz / (1.0f / l_dt));
        const float l_kSquared = l_k * l_k;
        const float l_norm = 1.0f + sqrtf(2.0f) * l_k + l_kSquared;

        m_samplingFrequency = 1.0f / l_dt;

        m_sinFilter.b0 = l_kSquared / l_norm;
        m_sinFilter.b1 = 2.0f * l_kSquared / l_norm;
        m_sinFilter.b2 = l_kSquared / l_norm;
        m_sinFilter.a1 = 2.0f * (l_kSquared - 1.0f) / l_norm;
        m_sinFilter.a2 = (1.0f - sqrtf(2.0f) * l_k + l_kSquared) / l_norm;

        m_cosFilter.b0 = m_sinFilter.b0;
        m_cosFilter.b1 = m_sinFilter.b1;
        m_cosFilter.b2 = m_sinFilter.b2;
        m_cosFilter.a1 = m_sinFilter.a1;
        m_cosFilter.a2 = m_sinFilter.a2;
    }

    CEncoder::~CEncoder()
    {
    };

    void CEncoder::serialCallbackENCODERcommand(char const * a, char * b)
    {
        uint8_t l_isActivate = 0;
        const uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res)
        {
            m_isActive = (l_isActivate >= 1);
            sprintf(b, "1");
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    void CEncoder::setSpeedReference(float f_speed)
    {
        for (size_t i = 0U; i < c_speedReferenceHistorySize - 1U; ++i)
        {
            m_speedReferenceHistory[i] = m_speedReferenceHistory[i + 1U];
        }

        m_speedReference = f_speed;
        m_speedReferenceHistory[c_speedReferenceHistorySize - 1U] = f_speed;
    }

    float CEncoder::readAngleDegrees()
    {
        const float l_rawAngleDegrees = m_pwm.dutycycle() * 360.0f;
        const float l_rawAngleRadians = l_rawAngleDegrees * PI_FLOAT / 180.0f;
        const float l_sin = sinf(l_rawAngleRadians);
        const float l_cos = cosf(l_rawAngleRadians);
        const float l_filteredSin = m_sinFilter.process(l_sin);
        const float l_filteredCos = m_cosFilter.process(l_cos);

        float l_angle = atan2f(l_filteredSin, l_filteredCos) * 180.0f / PI_FLOAT;
        if (l_angle < 0.0f)
        {
            l_angle += 360.0f;
        }

        return applyHysteresis(l_angle);
    }

    float CEncoder::readAngularSpeed()
    {
        if (!m_timerStarted)
        {
            return readAngularSpeedKalman();
        }

        return m_lastAngularSpeed;
    }

    float CEncoder::readAngularAcceleration()
    {
        if (!m_timerStarted)
        {
            (void)readAngularSpeedKalman();
        }

        return m_lastAngularAcceleration;
    }

    float CEncoder::getTotalDisplacementDegrees()
    {
        const float l_rawAngle = m_pwm.dutycycle() * 360.0f;

        if (!m_hasDisplacementReference)
        {
            m_lastDisplacementRawAngle = l_rawAngle;
            m_hasDisplacementReference = true;
            return m_totalDisplacement;
        }

        float l_delta = l_rawAngle - m_lastDisplacementRawAngle;

        if (l_delta > 180.0f)
        {
            l_delta -= 360.0f;
        }
        else if (l_delta < -180.0f)
        {
            l_delta += 360.0f;
        }

        m_totalDisplacement += l_delta;
        m_lastDisplacementRawAngle = l_rawAngle;

        return m_totalDisplacement;
    }

    void CEncoder::resetTravelDistance()
    {
        m_totalDisplacement = 0.0f;
        m_hasDisplacementReference = false;
        m_lastDisplacementRawAngle = 0.0f;
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

    float CEncoder::getLinearAcceleration()
    {
        if (!m_timerStarted)
        {
            (void)readAngularSpeedKalman();
        }

        return m_lastLinearAcceleration;
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
            m_previousRawAngle = m_pwm.dutycycle() * 360.0f;
            return 0.0f;
        }

        const float l_dt = static_cast<float>(l_nowUs - m_lastTimerUs) * 1e-6f;
        m_lastTimerUs = l_nowUs;

        if (l_dt <= 0.0f)
        {
            return m_lastAngularSpeed;
        }

        const float l_rawAngleDegrees = m_pwm.dutycycle() * 360.0f;
        const float l_wrapDiff = l_rawAngleDegrees - m_previousRawAngle;

        if (l_wrapDiff > 180.0f)
        {
            m_unwrapRevolutions--;
        }
        else if (l_wrapDiff < -180.0f)
        {
            m_unwrapRevolutions++;
        }

        m_previousRawAngle = l_rawAngleDegrees;

        const float l_measurementAngle = l_rawAngleDegrees + 360.0f * static_cast<float>(m_unwrapRevolutions);

        m_kalman.predict(l_dt);
        m_kalman.update(l_measurementAngle);

        const float l_previousAngularSpeed = m_lastAngularSpeed;
        float l_speedDegPerSec = applyHampel(m_kalman.speed);
        l_speedDegPerSec = applySpeedHysteresis(l_speedDegPerSec);

        const float l_rawAcceleration = (l_speedDegPerSec - l_previousAngularSpeed) / l_dt;
        m_lastAngularAcceleration = ACCELERATION_FILTER_ALPHA * l_rawAcceleration
                                  + (1.0f - ACCELERATION_FILTER_ALPHA) * m_lastAngularAcceleration;
        m_lastAngularSpeed = l_speedDegPerSec;

        // Keep the raw measured encoder speed visible while the car calibration is still in progress.
        // m_lastLinearSpeed = applyReferenceFilter(l_speedDegPerSec / DEGREE_PER_MM);
        m_lastLinearSpeed = convertAngularToLinear(l_speedDegPerSec);
        m_lastLinearAcceleration = convertAngularToLinear(l_rawAcceleration);

        // m_lastLinearSpeed = l_speedDegPerSec / DEGREE_PER_MM;
        // m_lastLinearAcceleration = m_lastAngularAcceleration / DEGREE_PER_MM;

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

    float CEncoder::applyHysteresis(float f_angle)
    {
        if (f_angle > m_lastAngle + m_angleHysteresis)
        {
            m_lastAngle = f_angle;
        }
        else if (f_angle < m_lastAngle - m_angleHysteresis)
        {
            m_lastAngle = f_angle;
        }
        else
        {
            f_angle = m_lastAngle;
        }

        return m_lastAngle = f_angle;
    }

    float CEncoder::applySpeedHysteresis(float f_speed)
    {
        if (f_speed > m_lastAngularSpeed + m_speedHysteresis)
        {
            m_lastAngularSpeed = f_speed;
        }
        else if (f_speed < m_lastAngularSpeed - m_speedHysteresis)
        {
            m_lastAngularSpeed = f_speed;
        }
        else
        {
            f_speed = m_lastAngularSpeed;
        }

        return m_lastAngularSpeed = f_speed;
    }

    float CEncoder::applyReferenceFilter(float f_speedMmPerSec) const
    {
        const float l_referenceMagnitude = fabsf(m_speedReference);
        if (l_referenceMagnitude < MIN_REFERENCE_FOR_RATIO_GATE)
        {
            return f_speedMmPerSec;
        }

        float l_minDifference = fabsf(f_speedMmPerSec - m_speedReferenceHistory[0]);

        for (size_t i = 1U; i < c_speedReferenceHistorySize; ++i)
        {
            const float l_difference = fabsf(f_speedMmPerSec - m_speedReferenceHistory[i]);
            if (l_difference < l_minDifference)
            {
                l_minDifference = l_difference;
            }
        }

        if ((l_minDifference / l_referenceMagnitude) > SPEED_GATE_RATIO)
        {
            return m_speedReference;
        }

        return f_speedMmPerSec;
    }

    float CEncoder::convertAngularToLinear(float f_angularQuantity) const
    {
        const float l_motorShaftRatio = 62.0f / 24.0f;
        const float l_shaftDiffRatio = 34.0f / 11.0f;
        const float l_wheelCircumference = WHEEL_DIAMETER * PI_FLOAT;
        const float l_gearRatio = l_motorShaftRatio * l_shaftDiffRatio;

        return -(f_angularQuantity * l_wheelCircumference) / (360.0f * l_gearRatio);
    }

    void CEncoder::publishSpeed()
    {
        char l_buffer[64];
        const int l_speedMmPerSec = static_cast<int>(roundf(m_lastLinearSpeed));

        snprintf(l_buffer, sizeof(l_buffer), "@encoder:%d;;\r\n", l_speedMmPerSec);
        m_serial.write(l_buffer, strlen(l_buffer));
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
