#pragma once

class PID
{
private:
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain

    float integral; // Integral of error
    float dt; // Time step

    float prev_error; // Previous error
    float prev_derivative; // Previous derivative

    float integral_limit; // Limit for integral windup

    float alpha; // Low-pass filter coefficient

public:
    PID(float kp, float ki, float kd, float dt = 1.0f, float integral_limit = 1000.0f, float alpha = 0.1f)
        : kp(kp), ki(ki), kd(kd), dt(dt), integral_limit(integral_limit), alpha(alpha),
          integral(0.0f), prev_error(0.0f), prev_derivative(0.0f) {}
    
    float compute(float& setpoint, float& measured_value);

    void reset()
    {
        integral = 0.0f;
        prev_error = 0.0f;
        prev_derivative = 0.0f;
    }

    void update(float new_kp, float new_ki, float new_kd, float new_alpha = 0.1f)
    {
        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
        alpha = new_alpha;
    }
};
