/*
 *	s4_port.h
 *	Subsystem 4 -- the few places that depend on things I could not see in the template.
 *	If the build complains, the fix is almost certainly in THIS file, nowhere else.
 */
#ifndef S4_PORT_H
#define S4_PORT_H


#include "s4_fmt.h"

#ifndef S4_PRINTF
#include "../app_console.h"
#if APP_CONSOLE
#define S4_PRINTF(...)		tm_printf((UB*)__VA_ARGS__)	/* the template console (USB serial) */
#else
#define S4_PRINTF(...)		((void)0)
#endif
#endif

/* RP2040 free-running 1 MHz timer, TIMERAWL (read-only register; no SDK needed) */
#define S4_US32()		(*(volatile unsigned int *)0x40054028u)

static inline uint32_t s4_now_ms(void)
{
	SYSTIM t;
	tk_get_otm(&t);
	return (uint32_t)t.lo;
}

#endif /* S4_PORT_H */
