#include "RovePIDController.h"

#include <cmath>


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



void RovePIDController::configOutputLimits(float max, float min) {
    configMaxOutput(max);
    configMinOutput(min);
}

void RovePIDController::configMaxOutput(float max) {
    m_maxOutput = max;
}

void RovePIDController::configMinOutput(float min) {
    m_minOutput = min;
}



void RovePIDController::reset() {
    m_firstLoop = true;
}

float RovePIDController::calculate(float target, float feedback, float timestamp) {
    float error = target - feedback;
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
            m_integral += error;
            if (m_integral > m_maxIntegralAccum) m_integral = m_maxIntegralAccum;
            else if (m_integral < -m_maxIntegralAccum) m_integral = -m_maxIntegralAccum;
        }
    }
    
    m_lastTimestamp = timestamp;
    m_lastError = error;

    return m_kP*error + m_kI*m_integral + m_kD*derivative;
}