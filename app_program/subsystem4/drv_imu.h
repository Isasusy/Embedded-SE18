/*
 *	drv_imu.h
 *	Subsystem 4 -- I2C driver for MPU-6050-family 6-axis IMUs (address 0x68 or 0x69).
 *
 *	OWNERSHIP (TEAM.md section 3): I2C0 and pins GP0 / GP1 belong to subsystem 4 (the Fritzing wiring).
 *	Only imu_task may call these functions.
 *
 *	Pins follow Figure 4.1 of the Design Review (task interaction diagram), not the Section 2 table.
 *	If the robot is really wired like the Fritzing picture (IMU on GP0/GP1) use I2C0 instead:
 *	    #define IMU_I2C i2c0   /  #define PIN_IMU_SDA 0   /  #define PIN_IMU_SCL 1
 *	(GP0 = SDA, GP1 = SCL) and remember GP0/GP1 are taken when the team builds with CRASH_REPORT=1.
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

bool	imu_hw_init(imu_delay_fn delay_ms);	/* bus + probe + configure; false if no IMU answers */
bool	imu_hw_read(imu_raw_t *out);		/* one burst read, ~0.4 ms; false on any I2C error   */
uint8_t	imu_hw_addr(void);
uint8_t	imu_hw_who_am_i(void);

#endif /* DRV_IMU_H */
