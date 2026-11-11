/*
 *	drv_motor.c -- motor PWM through the kernel board library (libbsp pwm_*). No Pico SDK.
 *	GP8/GP9 are PWM slice 4 (left), GP10/GP11 are slice 5 (right). Clock divider is left at its reset value 1.0.
 */
#include "subsystem2.h"
#include <bsp/libbsp.h>
#include "drv_motor.h"
#include "motion_config.h"

#define PWM_WRAP	12499u		/* 125 MHz / 12500 = 10 kHz */

/* make sure the PWM block is out of reset (RESETS.RESET bit 14); harmless if the kernel already did it */
static void pwm_unreset(void)
{
	volatile unsigned int *rst  = (volatile unsigned int *)0x4000C000u;	/* RESETS_RESET */
	volatile unsigned int *done = (volatile unsigned int *)0x4000C008u;	/* RESETS_RESET_DONE */
	*rst &= ~(1u << 14);
	while (!(*done & (1u << 14))) { }
}

static void pin_init(unsigned pin)
{
	pwm_set_pin(pin);
	pwm_set_wrap(pin, PWM_WRAP);
	pwm_set_cc(pin, 0);
	pwm_set_enabled(pin, TRUE);
}

static void one(unsigned pa, unsigned pb, int duty)
{
	unsigned level;
	if (duty > 1000)  duty = 1000;
	if (duty < -1000) duty = -1000;
	level = (unsigned)((duty < 0 ? -duty : duty) * (int)PWM_WRAP / 1000);
	pwm_set_cc(pa, duty > 0 ? level : 0);
	pwm_set_cc(pb, duty < 0 ? level : 0);
}

void motor_init(void)
{
	pwm_unreset();
	pin_init(MOTOR_L_A);
	pin_init(MOTOR_L_B);
	pin_init(MOTOR_R_A);
	pin_init(MOTOR_R_B);
}

void motor_set(int left, int right)
{
	one(MOTOR_L_A, MOTOR_L_B, MC_MOTOR_L_SIGN * left);
	one(MOTOR_R_A, MOTOR_R_B, MC_MOTOR_R_SIGN * right);
}

void motor_off(void)
{
	one(MOTOR_L_A, MOTOR_L_B, 0);
	one(MOTOR_R_A, MOTOR_R_B, 0);
}
