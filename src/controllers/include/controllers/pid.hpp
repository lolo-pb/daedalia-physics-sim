#pragma once

struct PidConfig {
    float proportional = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    float minimum_integral = 0.0f;
    float maximum_integral = 0.0f;
    float minimum_output = 0.0f;
    float maximum_output = 0.0f;
};

class Pid {
public:
    explicit Pid(const PidConfig &config = {});

    void Reset();
    float Update(float error, float measured_rate, float timestep);

private:
    PidConfig config_;
    float integral_ = 0.0f;
};
