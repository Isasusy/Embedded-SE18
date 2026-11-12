/*
 *	drv_button.h / drv_button.c -- the START button on GP20, read ONLY by vc_task.
 *
 *	Wiring, no external parts: one side of the button to GP20, the other side to GND.
 *	  button open    -> the RP2040's internal pull-up holds GP20 high -> "not pressed"
 *	  button pressed -> the button pulls GP20 to GND                  -> "pressed" (active low)
 *	So a cut wire, a missing button or a wrong pin reads "not pressed" and the car stays in its
 *	power-up STOP state instead of starting by itself.  Nothing moves while the button is open.
 *
 *	The pull resistor is the only thing this driver programs; the pin is claimed in TEAM.md
 *	section 2 (GP20, subsystem 5).
 */
#ifndef DRV_BUTTON_H
#define DRV_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

#define PIN_START_BTN	20		/* GP20, button to GND (change here and in TEAM.md) */

bool	button_init(void);		/* once, from subsystem5_init: input pin + internal pull resistor; false = bad pin */
bool	button_is_pressed(void);	/* the live level right now, no debounce (start-up test / debugger) */
bool	button_pressed(void);		/* true ONCE per press; call every VC tick (20 ms) */

#endif /* DRV_BUTTON_H */
