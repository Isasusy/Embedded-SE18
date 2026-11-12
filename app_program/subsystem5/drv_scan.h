/*
 *	drv_scan.h / drv_scan.c -- Obstacle Scan hardware, called ONLY from scan_task.
 *	Servo SG90 signal on GP15 (PWM 50 Hz, slice 7 channel B, 5 V supply).
 *	Ultrasonic HC-SR04: TRIG and ECHO pins below (if nothing answers at power-up, ultra_init() also tries the swapped
 *	pair and the other free pins, and prints what it found). If the real wires differ, change only these three numbers.
 */
#ifndef DRV_SCAN_H
#define DRV_SCAN_H

#define PIN_SERVO	15
#define PIN_US_TRIG	16
#define PIN_US_ECHO	17

void	servo_init(void);
void	servo_set_deg(int deg);		/* -90 .. +90, + = left, 0 = straight ahead */
void	ultra_init(void);		/* registers the default pins; if nothing answers, searches for the wiring (see below) */
int	ultra_ping_mm(void);		/* distance in mm, or -1 for no echo; blocks up to ~25 ms */
int	ultra_autodetect(void);		/* try every TRIG/ECHO pair among the free pins; 1 and the pins are kept if a sensor answers */
int	ultra_trig_pin(void);		/* pins in use now (the defaults, or the ones autodetect found) */
int	ultra_echo_pin(void);

/* Robo Pico on-board push buttons GP20 / GP21 (pressed = low). Read by vc_task (start / stop button). */
#define PIN_BTN_A	20
#define PIN_BTN_B	21
void	button_init(void);		/* both pins as inputs with pull-ups */
int	button_down(void);		/* 1 while either button is held */

#endif
