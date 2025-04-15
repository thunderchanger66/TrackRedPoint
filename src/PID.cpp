#include <PID.hpp>

float PID::compute(float& setpoint, float& measured_value)
{
    // Calculate error
    float error = setpoint - measured_value;

    // Integral term with anti-windup
    integral += error * dt;
    if (integral > integral_limit)
        integral = integral_limit;
    else if (integral < -integral_limit)
        integral = -integral_limit;

    // Derivative term with low-pass filter
    float derivative = (error - prev_error) / dt;
    derivative = alpha * derivative + (1.0f - alpha) * prev_derivative;

    // Update previous values
    prev_error = error;
    prev_derivative = derivative;

    // Compute output
    return kp * error + ki * integral + kd * derivative;
}