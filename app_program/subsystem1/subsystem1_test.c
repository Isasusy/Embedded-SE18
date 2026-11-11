/*
 *	subsystem1_test.c -- on-target test hook for "make TEST=1".
 *	No on-target tests written yet. The logic tests run on a PC: see testing/subsystem1/pc/.
 */
#include "subsystem1.h"

#if APP_TEST
EXPORT void subsystem1_test(void)
{
	tm_printf((UB*)"[TEST] subsystem 1: no on-target tests yet\n");
}
#endif
