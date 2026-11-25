/* s3_port.h -- subsystem 3: console print, microsecond timer, ms clock. Fix build problems here first. */
#ifndef S3_PORT_H
#define S3_PORT_H

#ifndef S3_PRINTF
#include "../app_console.h"
#if APP_CONSOLE
#define S3_PRINTF(...)		tm_printf((UB*)__VA_ARGS__)	/* the template console (USB serial) */
#else
#define S3_PRINTF(...)		((void)0)
#endif
#endif
/* RP2040 free-running 1 MHz timer, TIMERAWL (read-only register; no SDK needed) */
#define S3_US32()		(*(volatile unsigned int *)0x40054028u)
static inline uint32_t s3_now_ms(void) { SYSTIM t; tk_get_otm(&t); return (uint32_t)t.lo; }

#include "line_config.h"
#include "../subsystem2/drv_encoder.h"
#include "../subsystem2/motion_config.h"
#define S3_ODO_MM_PER_UNIT	(3.14159265f * MC_WHEEL_DIAM_MM / MC_TICKS_PER_REV / 2.0f)
/* distance the barcode sensor has travelled, in half encoder ticks */
static inline int32_t s3_odo_units(void)
{
	int32_t l, r;
	enc_read(&l, &r);
	return (l + r) - (int32_t)(2.0f * BAR_SENSOR_Y_MM * (float)(r - l) / MC_TRACK_MM);
}
#endif
