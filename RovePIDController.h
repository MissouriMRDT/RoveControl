#ifndef ROVEPIDCONTROLLER_H
#define ROVEPIDCONTROLLER_H

#include <cfloat>

class RovePIDController {

private:

    float m_kP = 0, m_kI = 0, m_kD = 0;
    float m_iZone = FLT_MAX, m_maxIntegralAccum = FLT_MAX;
    float m_maxOutput = FLT_MAX, m_minOutput = -FLT_MAX;
    float m_lastError, m_lastTimestamp, m_integral;
    bool m_firstLoop = true;

public:

    RovePIDController() {}
    RovePIDController(float kP, float kI, float kD) : m_kP(kP), m_kI(kI), m_kD(kD) {}

    void configPID(float kP, float kI, float kD);
    void configKP(float kP);
    void configKI(float kI);
    void configKD(float kD);

    void configIZone(float iZone);
    void configMaxIntegralAccum(float max);

    void configOutputLimits(float max, float min);
    void configMaxOutput(float max);
    void configMinOutput(float min);

    void reset();
    float calculate(float target, float feedback, float timestamp);

};

#endif