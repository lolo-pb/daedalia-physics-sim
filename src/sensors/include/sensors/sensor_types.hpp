#pragma once

struct SensorVector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct ImuSample {
    double timestamp_seconds = 0.0;
    SensorVector3 body_gyro_rad_per_second;
    SensorVector3 body_specific_force_meters_per_second_squared;
};

struct GpsSample {
    double timestamp_seconds = 0.0;
    SensorVector3 world_position_meters;
    SensorVector3 world_velocity_meters_per_second;
};

struct BarometerSample {
    double timestamp_seconds = 0.0;
    float pressure_pascals = 0.0f;
};

struct MagnetometerSample {
    double timestamp_seconds = 0.0;
    SensorVector3 body_magnetic_field_microteslas;
};
