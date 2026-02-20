#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>


/**
 * @brief Representation of a sensor readout value.
 *
 * The value is represented as having an integer and a fractional part,
 * and can be obtained using the formula val1 + val2 * 10^(-6). Negative
 * values also adhere to the above formula, but may need special attention.
 * Here are some examples of the value representation:
 *
 *      0.5: val1 =  0, val2 =  500000
 *     -0.5: val1 =  0, val2 = -500000
 *     -1.0: val1 = -1, val2 =  0
 *     -1.5: val1 = -1, val2 = -500000
 */
struct sensor_value {
	/** Integer part of the value. */
	int32_t val1;
	/** Fractional part of the value (in one-millionth parts). */
	int32_t val2;
};


/**
 * @brief Sensor channels.
 */
enum sensor_channel {
	/** Acceleration on the X axis, in m/s^2. */
	SENSOR_CHAN_ACCEL_X,
	/** Acceleration on the Y axis, in m/s^2. */
	SENSOR_CHAN_ACCEL_Y,
	/** Acceleration on the Z axis, in m/s^2. */
	SENSOR_CHAN_ACCEL_Z,
	/** Acceleration on the X, Y and Z axes. */
	SENSOR_CHAN_ACCEL_XYZ,
	/** Angular velocity around the X axis, in radians/s. */
	SENSOR_CHAN_GYRO_X,
	/** Angular velocity around the Y axis, in radians/s. */
	SENSOR_CHAN_GYRO_Y,
	/** Angular velocity around the Z axis, in radians/s. */
	SENSOR_CHAN_GYRO_Z,
	/** Angular velocity around the X, Y and Z axes. */
	SENSOR_CHAN_GYRO_XYZ,
    /** Magnetic field on the X axis, in Gauss. */
	SENSOR_CHAN_MAGN_X,
	/** Magnetic field on the Y axis, in Gauss. */
	SENSOR_CHAN_MAGN_Y,
	/** Magnetic field on the Z axis, in Gauss. */
	SENSOR_CHAN_MAGN_Z,
	/** Magnetic field on the X, Y and Z axes. */
	SENSOR_CHAN_MAGN_XYZ,
	/** Device die temperature in degrees Celsius. */
	SENSOR_CHAN_DIE_TEMP,
	/** Ambient temperature in degrees Celsius. */
	SENSOR_CHAN_AMBIENT_TEMP,
	/** Pressure in kilopascal. */
	SENSOR_CHAN_PRESS,

    SENSOR_CHAN_ALL,
};

/**
 * @brief The value of gravitational constant in micro m/s^2.
 */
#define SENSOR_G		9806650LL

/**
 * @brief The value of constant PI in micros.
 */
#define SENSOR_PI		3141592LL

/**
 * @brief Helper function for converting 10 micro degrees to radians.
 *
 * @param d The value (in 10 micro degrees) to be converted.
 * @param rad A pointer to a sensor_value struct, where the result is stored.
 */
static inline void sensor_10udegrees_to_rad(int32_t d, struct sensor_value *rad)
{
	rad->val1 = ((int64_t)d * SENSOR_PI / 180LL / 100000LL) / 1000000LL;
	rad->val2 = ((int64_t)d * SENSOR_PI / 180LL / 100000LL) % 1000000LL;
}

/**
 * @brief Helper function to convert acceleration from micro Gs to m/s^2
 *
 * @param ug The micro G value to be converted.
 * @param ms2 A pointer to a sensor_value struct, where the result is stored.
 */
static inline void sensor_ug_to_ms2(int32_t ug, struct sensor_value *ms2)
{
	ms2->val1 = ((int64_t)ug * SENSOR_G / 1000000LL) / 1000000LL;
	ms2->val2 = ((int64_t)ug * SENSOR_G / 1000000LL) % 1000000LL;
}

/**
 * @brief Helper function for converting struct sensor_value to float.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
static inline float sensor_value_to_float(const struct sensor_value *val)
{
	return (float)val->val1 + (float)val->val2 / 1000000;
}


#endif // __SENSOR_H__