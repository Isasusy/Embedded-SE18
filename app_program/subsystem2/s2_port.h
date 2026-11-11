/*
 *	s2_port.h
 *	Subsystem 2 -- the few places that depend on things I could not see in the template.
 *	If the build complains, the fix is almost certainly in THIS file.
 */
#ifndef S2_PORT_H
#define S2_PORT_H



#ifndef S2_PRINTF
#include "../app_console.h"
#if APP_CONSOLE
#define S2_PRINTF(...)		tm_printf((UB*)__VA_ARGS__)	/* the template console (USB serial) */
#else
#define S2_PRINTF(...)		((void)0)	/* console off (APP_CONSOLE 0 in app_console.h) */
#endif
#endif

/* RP2040 free-running 1 MHz timer, TIMERAWL (read-only register; no SDK needed) */
#define S2_US32()		(*(volatile unsigned int *)0x40054028u)

static inline uint32_t s2_now_ms(void)
{
	SYSTIM t;
	tk_get_otm(&t);
	return (uint32_t)t.lo;
}

#endif
