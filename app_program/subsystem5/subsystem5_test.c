/*
 *	subsystem5_test.c -- on-target test hook for "make TEST=1".
 *	No on-target tests written yet. The logic tests run on a PC: see testing/subsystem5/pc/.
 */
#include "subsystem5.h"

#if APP_TEST
EXPORT void subsystem5_test(void)
{
	tm_printf((UB*)"[TEST] subsystem 5: no on-target tests yet\n");
}
#endif
