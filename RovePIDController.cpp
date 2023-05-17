#include "RovePIDController.h"

#include <cmath>


void RovePIDController::configPID(const float& kP, const float& kI, const float& kD) {
    configKP(kP);
    configKI(kI);
    configKD(kD);
}

void RovePIDController::configKP(const float& kP) {
    m_kP = kP;
}

void RovePIDController::configKI(const float& kI) {
    m_kI = kI;
}

void RovePIDController::configKD(const float& kD) {
    m_kD = kD;
}



void RovePIDController::configIZone(const float& iZone) {
    m_iZone = iZone;
}

void RovePIDController::configMaxIntegralAccum(const float& max) {
    m_maxIntegralAccum = max;
}



void RovePIDController::configOutputLimits(const float& min, const float& max) {
    configMinOutput(min);
    configMaxOutput(max);
}

void RovePIDController::configMaxOutput(const float& max) {
    m_maxOutput = max;
}

void RovePIDController::configMinOutput(const float& min) {
    m_minOutput = min;
}



void RovePIDController::enableContinuousFeedback(const float& minFeedback, const float& maxFeedback) {
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

float RovePIDController::calculate(const float& target, const float& feedback, const float& timestamp) const {
    float error = target - feedback;
    if (m_continuous) {
        float modulus = m_maxFeedback - m_minFeedback;
        error = std::fmod(error, modulus);

        if (error > modulus/2) error -= modulus;
        else if (error < -modulus/2) error += modulus;
    }

    float derivative;
    if (m_firstLoop) {
        derivative = 0;
        m_integral = 0;
        m_firstLoop = false;
    }
    else {
        float dt = timestamp - m_lastTimestamp;
        if (dt == 0) derivative = 0;
        else derivative = (error - m_lastError) / dt;

        if (std::abs(error) > m_iZone) {
            m_integral = 0;
        }
        else {
            m_integral += error * dt;
            if (m_integral > m_maxIntegralAccum) m_integral = m_maxIntegralAccum;
            else if (m_integral < -m_maxIntegralAccum) m_integral = -m_maxIntegralAccum;
        }
    }
    
    m_lastTimestamp = timestamp;
    m_lastError = error;

    float output = m_kP*error + m_kI*m_integral + m_kD*derivative;
    
    if(output > m_maxOutput) return m_maxOutput;
    if(output < m_minOutput) return m_minOutput;
    return output;
}