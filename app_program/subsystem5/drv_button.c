/*
 *	drv_button.c -- the START button on GP20.
 *
 *	No kernel object, nothing blocks: button_pressed() is one register read plus a few
 *	comparisons, called from vc_task once per 20 ms tick.  Debouncing is done here so the
 *	Vehicle Controller only ever sees clean press edges.
 */
#include "subsystem5.h"
#include <bsp/libbsp.h>
#include "drv_button.h"
#include "vc_config.h"
#include "s5_port.h"

static bool	s_level;	/* debounced level: true = pressed            */
static bool	s_raw;		/* raw level at the previous sample           */
static uint32_t	s_t_raw;	/* when s_raw last changed, ms                */

/* the pin level as "pressed / not pressed", honouring VC_START_BTN_ACTIVE_LOW */
static bool read_pressed(void)
{
	UINT v = gpio_get_val(PIN_START_BTN);
#if VC_START_BTN_ACTIVE_LOW
	return v == 0;			/* button to GND: pressed = low               */
#else
	return v != 0;			/* button to 3V3: pressed = high              */
#endif
}

/* internal pull resistor on the pad.
   PADS_BANK0 GPIOn = 0x4001C004 + 4n;  IE = bit 6, PUE = bit 3, PDE = bit 2, schmitt = bit 1.
   The pad's own IE is set as well, so the level reaches the SIO input whatever the kernel left. */
static void pull(unsigned pin, bool up)
{
	volatile unsigned int *pad = (volatile unsigned int *)(0x4001C004u + 4u * pin);

	if (up)
		*pad = (*pad | (1u << 6) | (1u << 3) | (1u << 1)) & ~(1u << 2);
	else
		*pad = (*pad | (1u << 6) | (1u << 2) | (1u << 1)) & ~(1u << 3);
}

bool button_init(void)
{
	ER er = gpio_set_pin(PIN_START_BTN, GPIO_MODE_IN);	/* input, SIO function, output driver off */
	if (er < E_OK) return false;
	pull(PIN_START_BTN, VC_START_BTN_ACTIVE_LOW != 0);

	/* Whatever the pin reads now is the resting level.  A button that is already held at
	   power-up is therefore NOT a press: the car still needs a fresh press to start. */
	s_raw = s_level = read_pressed();
	s_t_raw = s5_now_ms();
	return true;
}

bool button_is_pressed(void)
{
	return read_pressed();
}

/*
 * One press = one true.  The level must be stable for VC_START_BTN_DEBOUNCE_MS before it is
 * taken, so contact bounce (and the noise on an unshielded wire next to the motors) is ignored.
 * With the default 25 ms and a 20 ms tick, a press is reported on the second tick, ~40 ms.
 */
bool button_pressed(void)
{
	bool raw = read_pressed();
	uint32_t now = s5_now_ms();

	if (raw != s_raw) {		/* the pin changed: (re)start the debounce window */
		s_raw = raw;
		s_t_raw = now;
		return false;
	}
	if (raw == s_level)		/* stable and already known: nothing new */
		return false;
	if (now - s_t_raw < VC_START_BTN_DEBOUNCE_MS)
		return false;		/* still bouncing */

	s_level = raw;			/* stable for long enough: accept it */
	return raw;			/* true once, on the press edge only */
}
