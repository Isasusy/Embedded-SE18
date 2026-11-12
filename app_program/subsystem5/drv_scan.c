/*
 *	drv_scan.c -- servo (PWM) and ultrasonic (GPIO) through the kernel board library. No Pico SDK.
 *	GP15 is PWM slice 7, channel B (any pin works: the slice is computed from the pin number). libbsp has no clock-divider call, so the divider register is written directly.
 */
#include "subsystem5.h"
#include <bsp/libbsp.h>
#include "drv_scan.h"
#include "vc_config.h"
#include "s5_port.h"

#define SERVO_WRAP	62499u			/* (125 MHz / 40) / 62500 = 50 Hz */
#define PWM_DIV_REG(slice)	(*(volatile unsigned int *)(0x40050000u + 0x14u * (slice) + 0x04u))

static void pwm_unreset(void)			/* RESETS.RESET bit 14 = PWM; harmless if already released */
{
	volatile unsigned int *rst  = (volatile unsigned int *)0x4000C000u;
	volatile unsigned int *done = (volatile unsigned int *)0x4000C008u;
	*rst &= ~(1u << 14);
	while (!(*done & (1u << 14))) { }
}

void servo_init(void)
{
	pwm_unreset();
	pwm_set_pin(PIN_SERVO);
	PWM_DIV_REG((PIN_SERVO >> 1) & 7u) = 40u << 4;		/* DIV is 8.4 fixed point: 40.0 */
	pwm_set_wrap(PIN_SERVO, SERVO_WRAP);
	pwm_set_cc(PIN_SERVO, 0);
	pwm_set_enabled(PIN_SERVO, TRUE);
	servo_set_deg(0);
}

void servo_set_deg(int deg)
{
#if SERVO_REVERSE
	deg = -deg;
#endif
	deg += SERVO_TRIM_DEG;
	if (deg < -90) deg = -90;
	if (deg > 90) deg = 90;
	/* -90 deg -> SERVO_US_MIN, +90 deg -> SERVO_US_MAX. Mounted so that + is the robot's left; flip here if not. */
	unsigned us = SERVO_US_MIN + (unsigned)((deg + 90) * (SERVO_US_MAX - SERVO_US_MIN) / 180);
	pwm_set_cc(PIN_SERVO, (us * 3125u) / 1000u);		/* 3.125 ticks per microsecond */
}

/* ---- ultrasonic: plain GPIO through the RP2040 registers (no SDK, no libbsp GPIO semantics to guess) ---- */
#define IO_CTRL(p)	(*(volatile unsigned int *)(0x40014000u + 8u * (p) + 4u))	/* IO_BANK0 GPIOn_CTRL, FUNCSEL 5 = SIO */
#define PAD(p)		(*(volatile unsigned int *)(0x4001C000u + 4u + 4u * (p)))	/* PADS_BANK0 GPIOn */
#define SIO_IN		(*(volatile unsigned int *)0xD0000004u)
#define SIO_OUT_SET	(*(volatile unsigned int *)0xD0000014u)
#define SIO_OUT_CLR	(*(volatile unsigned int *)0xD0000018u)
#define SIO_OE_SET	(*(volatile unsigned int *)0xD0000024u)
#define SIO_OE_CLR	(*(volatile unsigned int *)0xD0000028u)

static int us_trig = PIN_US_TRIG, us_echo = PIN_US_ECHO;

/* pins the search may use: free on this robot (not IMU 0/1, encoders 2-5, motors 8-11, servo 15,
   the GP20 START button, IR 26-28).  Never add GP20: the search would drive the button's pin. */
static const unsigned char cand[] = { 16, 17, 18, 19, 21, 22, 12, 13, 14, 6, 7 };

static void pin_out(int p)
{
	PAD(p) = (1u << 6) | (1u << 1) | (1u << 4);		/* input enable, schmitt, 4 mA drive */
	IO_CTRL(p) = 5u;
	SIO_OUT_CLR = 1u << p;
	SIO_OE_SET = 1u << p;
}

static void pin_in(int p)
{
	SIO_OE_CLR = 1u << p;
	PAD(p) = (1u << 6) | (1u << 1) | (1u << 2);		/* input enable, schmitt, pull-down (the sensor drives it) */
	IO_CTRL(p) = 5u;
}

static void wait_us(uint32_t us)
{
	uint32_t t0 = S5_US32();
	while ((uint32_t)(S5_US32() - t0) < us) { }
}

/* trigger and time the echo pulse, in microseconds; -1 = no echo or too long */
static int ping_us(int t, int e)
{
	unsigned int tb = 1u << t, eb = 1u << e;
	SIO_OUT_SET = tb;
	wait_us(12);
	SIO_OUT_CLR = tb;

	uint32_t t0 = S5_US32();
	while (!(SIO_IN & eb))					/* wait for the echo pulse to start */
		if ((uint32_t)(S5_US32() - t0) > 6000u) return -1;
	uint32_t t1 = S5_US32();
	while (SIO_IN & eb)					/* ... and end (25 ms = about 4 m) */
		if ((uint32_t)(S5_US32() - t1) > 25000u) return -1;
	return (int)(S5_US32() - t1);
}

int ultra_ping_mm(void)
{
	int us = ping_us(us_trig, us_echo);
	return us < 0 ? -1 : (int)((unsigned)us * 343u / 2000u);	/* 0.343 mm/us, there and back */
}

/* Does an echo line rise after a trigger on (t, e)? Used only to find the wiring. */
static int echo_rises(int t, int e)
{
	unsigned int tb = 1u << t, eb = 1u << e;
	int hits = 0;

	pin_out(t);
	pin_in(e);
	for (int k = 0; k < 3; k++) {
		wait_us(2000);
		if (SIO_IN & eb) continue;				/* already high: not an idle echo line */
		SIO_OUT_SET = tb;
		wait_us(12);
		SIO_OUT_CLR = tb;
		uint32_t t0 = S5_US32();
		while (!(SIO_IN & eb))
			if ((uint32_t)(S5_US32() - t0) > 3000u) break;
		if (SIO_IN & eb) {
			hits++;
			uint32_t t1 = S5_US32();			/* let the sensor finish its echo before the next trigger */
			while ((SIO_IN & eb) && (uint32_t)(S5_US32() - t1) < 40000u) { }
			wait_us(20000);
		}
	}
	return hits >= 2;
}

int ultra_autodetect(void)
{
	int n = (int)(sizeof cand / sizeof cand[0]);

	/* the configured pair first, then its swap, then everything else */
	if (echo_rises(PIN_US_TRIG, PIN_US_ECHO)) { us_trig = PIN_US_TRIG; us_echo = PIN_US_ECHO; return 1; }
	if (echo_rises(PIN_US_ECHO, PIN_US_TRIG)) { us_trig = PIN_US_ECHO; us_echo = PIN_US_TRIG; return 1; }
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++) {
			if (i == j) continue;
			if (echo_rises(cand[i], cand[j])) { us_trig = cand[i]; us_echo = cand[j]; return 1; }
		}
	pin_out(us_trig);					/* leave the defaults configured */
	pin_in(us_echo);
	return 0;
}

int ultra_trig_pin(void) { return us_trig; }
int ultra_echo_pin(void) { return us_echo; }

void ultra_init(void)
{
	pin_out(us_trig);
	pin_in(us_echo);
	int ok = 0;
	for (int k = 0; k < 3; k++) {				/* a quick check on the configured pins */
		if (ultra_ping_mm() > 0) ok++;
		wait_us(30000);
	}
	if (!ok) {
		S5_PRINTF("[S5] ultrasonic: no echo on TRIG GP%d / ECHO GP%d, searching the other pins ...\n", us_trig, us_echo);
		if (ultra_autodetect())
			S5_PRINTF("[S5] ultrasonic FOUND: TRIG=GP%d ECHO=GP%d (set PIN_US_TRIG / PIN_US_ECHO in drv_scan.h to these)\n", us_trig, us_echo);
		else
			S5_PRINTF("[S5] ultrasonic: nothing answered on any free pin. Check the sensor's 5V and GND and that it is plugged in.\n");
	} else {
		S5_PRINTF("[S5] ultrasonic ok on TRIG GP%d / ECHO GP%d\n", us_trig, us_echo);
	}
}

/* ---- start / stop button ---- */
void button_init(void)
{
	const int pins[2] = { PIN_BTN_A, PIN_BTN_B };
	for (int i = 0; i < 2; i++) {
		int p = pins[i];
		SIO_OE_CLR = 1u << p;
		PAD(p) = (1u << 6) | (1u << 1) | (1u << 3);	/* input enable, schmitt, pull-UP (the button pulls to GND) */
		IO_CTRL(p) = 5u;
	}
}

int button_down(void)
{
	unsigned in = SIO_IN;
	return !(in & (1u << PIN_BTN_A)) || !(in & (1u << PIN_BTN_B));
}
