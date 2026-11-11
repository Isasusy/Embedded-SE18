/*
 *	subsystem2_test.c -- on-target test hook for "make TEST=1".
 *	No on-target tests written yet. The logic tests run on a PC: see testing/subsystem2/pc/.
 */
#include "subsystem2.h"

#if APP_TEST
EXPORT void subsystem2_test(void)
{
	tm_printf((UB*)"[TEST] subsystem 2: no on-target tests yet\n");
}
#endif
