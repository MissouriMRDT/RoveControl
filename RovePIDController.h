#ifndef ROVEPIDCONTROLLER_H
#define ROVEPIDCONTROLLER_H

#include <cfloat>
#include <cstdint>

class RovePIDController {

private:

    float m_kP = 0, m_kI = 0, m_kD = 0;
    float m_iZone = FLT_MAX, m_maxIntegralAccum = FLT_MAX;
    float m_maxOutput = FLT_MAX, m_minOutput = -FLT_MAX;
    mutable float m_lastError, m_integral;
    mutable bool m_firstLoop = true;
    mutable uint32_t m_lastTimestamp;

    bool m_continuous = false;
    float m_maxFeedback, m_minFeedback;

public:

    /**
     * @brief Construct a new RovePIDController object.
     * 
     */
    RovePIDController() {}

    /**
     * @brief Construct a new RovePIDController object.
     * 
     * @param kP 
     * @param kI 
     * @param kD 
     */
    RovePIDController(const float kP, const float kI, const float kD) : m_kP(kP), m_kI(kI), m_kD(kD) {}


    /**
     * @brief Configure the gains of the PID Controller.
     * 
     * @param kP 
     * @param kI 
     * @param kD 
     */
    void configPID(const float kP, const float kI, const float kD);

    /**
     * @brief Configure the proportional gain of the PID controller.
     * 
     * @param kP 
     */
    void configKP(const float kP);

    /**
     * @brief Configure the integral gain of the PID controller.
     * 
     * @param kI 
     */
    void configKI(const float kI);

    /**
     * @brief Configure the derivative gain of the PID controller.
     * 
     * @param kD 
     */
    void configKD(const float kD);


    /**
     * @brief Configure the integral zone of the PID controller. If the absolute error is greater than this value
     * when calculate() is called, the integral will be reset to 0.
     * 
     * @param iZone [0, FLT_MAX], defaults to FLT_MAX.
     */
    void configIZone(const float iZone);

    /**
     * @brief Configure the maximum integral accumulation of the PID controller. If the integral is greater than this
     * value when calculate() is called, it will be set to equal this value.
     * 
     * @param max [0, FLT_MAX], defaults to FLT_MAX.
     */
    void configMaxIntegralAccum(const float max);


    /**
     * @brief Configure the maximum and minimum values returned by calculate().
     * 
     * @param min 
     * @param max 
     */
    void configOutputLimits(const float min, const float max);

    /**
     * @brief Configure the maximum value returned by calculate().
     * 
     * @param max 
     */
    void configMaxOutput(const float max);

    /**
     * @brief Configure the minimum value returned by calculate().
     * 
     * @param min 
     */
    void configMinOutput(const float min);


    /**
     * @brief Enable wrapping of feedback with the given range. For 360 degree rotation, pass (0, 360).
     * 
     * @param minFeedback Smaller representation of wrapping point.
     * @param maxFeedback Larger representation of wrapping point.
     */
    void enableContinuousFeedback(const float minFeedback, const float maxFeedback);

    /**
     * @brief Disable wrapping of feedback; disabled by default.
     */
    void disableContinuousFeedback();

    /**
     * @brief Read the last error in position.
     * 
     * @return Last error.
     */
    float error() const;


    /**
     * @brief Reset the PID controller. When calculate() is next called, the derivative and integral terms will be 0.
     * 
     */
    void reset() const;

    /**
     * @brief Calculate the output of the PID controller.
     * 
     * @param target
     * @param feedback
     * @return The output of the PID controller, bounded by the configured maximum and minimum values. 
     */
    float calculate(const float target, const float feedback) const;

};

#endif