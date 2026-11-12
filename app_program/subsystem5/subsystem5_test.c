/*
 *	subsystem5_test.c -- on-target tests for "make TEST=1".
 *	The logic tests run on a PC: see testing/subsystem5/pc/.
 */
#include "subsystem5.h"
#include "drv_button.h"
#include "../test_report.h"

#if APP_TEST
EXPORT void subsystem5_test(void)
{
	TEST_INIT();

	/* S5-U01: the GP20 START button must read "not pressed" at power-up.
	   Catches a stuck button, a button wired to 3V3, the wrong pin, and a missing internal
	   pull-up (a floating GP20 reads 0 = pressed, so the car would not start at all). */
	TEST_CHECK("S5-U01", !button_is_pressed(), "GP20 start button released at power-up");

	TEST_SUMMARY("subsystem5");
}
#endif
