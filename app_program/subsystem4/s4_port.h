/*
 *	s4_port.h
 *	Subsystem 4 -- the few places that depend on things I could not see in the template.
 *	If the build complains, the fix is almost certainly in THIS file, nowhere else.
 */
#ifndef S4_PORT_H
#define S4_PORT_H


#include "s4_fmt.h"

/* ---- 1. Console output. Change if the template uses another call (e.g. tm_printf). ---- */
#ifndef S4_PRINTF
#include "../app_console.h"
#if APP_CONSOLE
#define S4_PRINTF(...)		tm_printf((UB*)__VA_ARGS__)	/* the template console (USB serial) */
#else
#define S4_PRINTF(...)		((void)0)	/* console off (APP_CONSOLE 0 in app_console.h) */
#endif
#endif

/* ---- 2. Microsecond counter for jitter / execution-time logging (assumption A6).
 *         The Pico SDK's free-running 1 MHz timer, read-only, safe from any task. ---- */
/* RP2040 free-running 1 MHz timer, TIMERAWL (read-only register; no SDK needed) */
#define S4_US32()		(*(volatile unsigned int *)0x40054028u)

/* ---- 3. Milliseconds since boot, for event timestamps. T-Kernel "operating time". ---- */
static inline uint32_t s4_now_ms(void)
{
	SYSTIM t;
	tk_get_otm(&t);
	return (uint32_t)t.lo;	/* low 32 bits of the millisecond count; wrap-safe for differences */
}

#endif /* S4_PORT_H */
