#include "RovePIDController.h"

#include "Arduino.h"

void RovePIDController::configPID(float kP, float kI, float kD) {
    configKP(kP);
    configKI(kI);
    configKD(kD);
}

void RovePIDController::configKP(float kP) {
    m_kP = kP;
}

void RovePIDController::configKI(float kI) {
    m_kI = kI;
}

void RovePIDController::configKD(float kD) {
    m_kD = kD;
}



void RovePIDController::configIZone(float iZone) {
    m_iZone = iZone;
}

void RovePIDController::configMaxIntegralAccum(float max) {
    m_maxIntegralAccum = max;
}



void RovePIDController::configOutputLimits(float min, float max) {
    configMinOutput(min);
    configMaxOutput(max);
}

void RovePIDController::configMaxOutput(float max) {
    m_maxOutput = max;
}

void RovePIDController::configMinOutput(float min) {
    m_minOutput = min;
}

void RovePIDController::configOffset(float offset) {
    m_offset = offset;
}

void RovePIDController::enableContinuousFeedback(float minFeedback, float maxFeedback) {
    m_minFeedback = minFeedback;
    m_maxFeedback = maxFeedback;
    m_continuous = true;
}

void RovePIDController::disableContinuousFeedback() {
    m_continuous = false;
}

float RovePIDController::error() const {
    return m_lastError;
}


void RovePIDController::reset() const {
    m_firstLoop = true;
}

float RovePIDController::calculate(float target, float feedback) const {
    uint32_t timestamp, dt;

    timestamp = millis();

    target -= m_offset;
    feedback -= m_offset;
    target = fmod(target, 360.0);
    feedback = fmod(feedback, 360.0);
    if (target < 0) target += 360.0;
    if (feedback < 0) feedback += 360.0;

    float error = target - feedback;
    //Subtract offset to make seam inside no go zone

    if (m_continuous) {
        float modulus = m_maxFeedback - m_minFeedback;
        error = fmod(error, modulus);

        if (error > modulus/2) error -= modulus;
        else if (error < -modulus/2) error += modulus;
    }

    float derivative = 0;
    if (m_firstLoop) {
        m_integral = 0;
        m_firstLoop = false;
    }
    else {
        dt = timestamp - m_lastTimestamp;

        if (dt != 0) {
            derivative = (error - m_lastError) / dt;
        }

        if (abs(error) > m_iZone) {
            m_integral = 0;
        }
        else {
            m_integral += error * dt;
            if (m_integral > m_maxIntegralAccum) {
                m_integral = m_maxIntegralAccum;
            } else if (m_integral < -m_maxIntegralAccum) {
                m_integral = -m_maxIntegralAccum;
            }
        }
    }
    
    m_lastTimestamp = timestamp;
    m_lastError = error;

    float output = m_kP*error + m_kI*m_integral + m_kD*derivative;
    
    if(output > m_maxOutput) return m_maxOutput;
    if(output < m_minOutput) return m_minOutput;
    return output;
}