/*
 *	subsystem4_test.c -- on-target test hook for "make TEST=1".
 *	No on-target tests written yet. The logic tests run on a PC: see testing/subsystem4/pc/.
 */
#include "subsystem4.h"

#if APP_TEST
EXPORT void subsystem4_test(void)
{
	tm_printf((UB*)"[TEST] subsystem 4: no on-target tests yet\n");
}
#endif
