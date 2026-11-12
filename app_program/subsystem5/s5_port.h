/* s5_port.h -- subsystem 5: console print, microsecond timer, ms clock, hardware watchdog. Fix build problems here first. */
#ifndef S5_PORT_H
#define S5_PORT_H

#ifndef S5_PRINTF
#include "../app_console.h"
#if APP_CONSOLE
#define S5_PRINTF(...)		tm_printf((UB*)__VA_ARGS__)	/* the template console (USB serial) */
#else
#define S5_PRINTF(...)		((void)0)	/* console off (APP_CONSOLE 0 in app_console.h) */
#endif
#endif
/* RP2040 free-running 1 MHz timer, TIMERAWL (read-only register; no SDK needed) */
#define S5_US32()		(*(volatile unsigned int *)0x40054028u)
static inline uint32_t s5_now_ms(void) { SYSTIM t; tk_get_otm(&t); return (uint32_t)t.lo; }

/* RP2040 hardware watchdog by register (no SDK). Used only when VC_WATCHDOG_ENABLE = 1. UNTESTED on the bench:
   assumes the 12 MHz crystal is the reference clock; the counter decrements twice per us tick (RP2040-E1). */
#define S5_WDT_REG(off)		(*(volatile unsigned int *)(0x40058000u + (off)))
#define S5_PSM_WDSEL		(*(volatile unsigned int *)0x40010008u)	/* PSM.WDSEL: what a watchdog timeout resets */
static inline void s5_wdt_feed(unsigned ms) { S5_WDT_REG(0x04) = ms * 2000u; }		/* LOAD */
static inline void s5_wdt_enable(unsigned ms)
{
	S5_WDT_REG(0x00) &= ~(1u << 30);		/* CTRL.ENABLE off while configuring */
	/* WDSEL resets to 0 = a timeout resets NOTHING. Select every block except the two oscillators (bit 0 ROSC,
	   bit 1 XOSC), the same as the Pico SDK's watchdog_enable(), so a timeout really reboots the chip. */
	S5_PSM_WDSEL = 0x0001FFFFu & ~0x3u;
	S5_WDT_REG(0x2C) = (1u << 9) | 12u;		/* TICK: enable, 12 cycles of clk_ref = 1 us */
	s5_wdt_feed(ms);
	S5_WDT_REG(0x00) |= (1u << 30);			/* CTRL.ENABLE */
}
#endif
