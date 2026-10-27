#pragma once

struct SensorVector3 {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

constexpr SensorVector3 operator-(const SensorVector3 &vector) {
	return {-vector.x, -vector.y, -vector.z};
}

using vec3d = SensorVector3;

// Body axes: X forward, Y right, Z down. World samples use local NED.
struct ImuSample {
	double timestamp_seconds = 0.0;
	SensorVector3 body_gyro_rad_per_second;
	SensorVector3 body_specific_force_ft_per_second_squared;
};
struct GpsSample {
	double timestamp_seconds = 0.0;
	SensorVector3 position_ned_ft;
	SensorVector3 velocity_ned_ft_per_second;
};
struct BarometerSample {
	double timestamp_seconds = 0.0;
	double pressure_psf = 0.0;
};
struct MagnetometerSample {
	double timestamp_seconds = 0.0;
	SensorVector3 body_magnetic_field_microteslas;
};
