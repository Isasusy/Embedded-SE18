/*
 *	drv_motor.h / drv_motor.c
 *	Subsystem 2 -- Robo Pico motor driver outputs.  Called ONLY from motion_task (the only motor writer).
 *	Robo Pico: left motor M1 = GP8 (IN1A) / GP9 (IN1B).  Right motor M2 = GP10 (IN2A) / GP11 (IN2B).
 *	Forward = PWM on A, B low.  Reverse = PWM on B, A low.  PWM 10 kHz.
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#define MOTOR_L_A	8
#define MOTOR_L_B	9
#define MOTOR_R_A	10
#define MOTOR_R_B	11

void motor_init(void);			/* pins to PWM, all outputs 0 */
void motor_set(int left, int right);	/* duty -1000..1000 per wheel (forward positive) */
void motor_off(void);			/* both wheels free-running, outputs 0 */

#endif
