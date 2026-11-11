/* s3_port.h -- subsystem 3: console print, microsecond timer, ms clock. Fix build problems here first. */
#ifndef S3_PORT_H
#define S3_PORT_H

#ifndef S3_PRINTF
#include "../app_console.h"
#if APP_CONSOLE
#define S3_PRINTF(...)		tm_printf((UB*)__VA_ARGS__)	/* the template console (USB serial) */
#else
#define S3_PRINTF(...)		((void)0)	/* console off (APP_CONSOLE 0 in app_console.h) */
#endif
#endif
/* RP2040 free-running 1 MHz timer, TIMERAWL (read-only register; no SDK needed) */
#define S3_US32()		(*(volatile unsigned int *)0x40054028u)
static inline uint32_t s3_now_ms(void) { SYSTIM t; tk_get_otm(&t); return (uint32_t)t.lo; }
#endif
