/*
 *	subsystem3_test.c -- on-target test hook for "make TEST=1".
 *	No on-target tests written yet. The logic tests run on a PC: see testing/subsystem3/pc/.
 */
#include "subsystem3.h"

#if APP_TEST
EXPORT void subsystem3_test(void)
{
	tm_printf((UB*)"[TEST] subsystem 3: no on-target tests yet\n");
}
#endif
