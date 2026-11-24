/*
 *	drv_imu.h
 *	Subsystem 4 -- I2C driver for MPU-6050-family 6-axis IMUs (address 0x68 or 0x69).
 *
 *	OWNERSHIP (TEAM.md section 3): I2C0 and pins GP0 / GP1 belong to subsystem 4 (the Fritzing wiring).
 *	Only imu_task may call these functions.
 *
 *	The bus is the kernel's I2C device "iica" (I2C0). PIN_IMU_SDA / PIN_IMU_SCL below are only used to set the pin
 *	function and pull-ups; the kernel's pins are set in i2c_cnf_sysdep.h. GP0/GP1 are taken when the team builds
 *	with CRASH_REPORT=1.
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdint.h>
#include <stdbool.h>

#define PIN_IMU_SDA	0	/* I2C0 SDA (Fritzing wiring) */
#define PIN_IMU_SCL	1	/* I2C0 SCL */

typedef struct {
	float ax_g, ay_g, az_g;		/* sensor frame */
	float gx_dps, gy_dps, gz_dps;	/* sensor frame */
} imu_raw_t;

typedef void (*imu_delay_fn)(uint32_t ms);	/* lets the task keep its ALIVE flag set while waiting */

/* Supported chips, probed in this order:
 *   IMU_CHIP_MPU     MPU-6050 / 6500 / 9250 family at 0x68 / 0x69 (accelerometer + gyroscope)
 *   IMU_CHIP_LSM303  ST LSM303 (DLHC / AGR, e.g. GY-511) or LIS3DH accelerometer at 0x19 / 0x18 - NO gyroscope,
 *                    gyro readings are always 0. The LSM303's compass (0x1E) is not used. */
typedef enum { IMU_CHIP_NONE, IMU_CHIP_MPU, IMU_CHIP_LSM303 } imu_chip_t;

bool	imu_hw_init(imu_delay_fn delay_ms);	/* bus + probe + configure; false if no IMU answers */
bool	imu_hw_read(imu_raw_t *out);		/* one burst read, ~0.4 ms; false on any I2C error   */
uint8_t	imu_hw_addr(void);
uint8_t	imu_hw_who_am_i(void);
imu_chip_t imu_hw_chip(void);
bool	imu_hw_has_gyro(void);			/* false for the LSM303: the algorithm then uses the accelerometer for pitch */

#endif /* DRV_IMU_H */
