#include <stdio.h>
#include <string.h>
#include "../../../app_program/subsystem1/telemetry.h"
static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
int main(void)
{
	char b[160];
	robot_status_t s = { VC_LINE_FOLLOW, 148, 22.34f, FAULT_NONE, 123456 };
	int n = tm_format_status(b, sizeof b, &s);
	CHECK(n == (int)strlen(b), "length matches");
	CHECK(!strcmp(b, "{\"state\":\"LINE_FOLLOW\",\"speed\":148,\"max_hump_mm\":22.3,\"fault\":\"NONE\",\"t\":123456}"), "status JSON: %s", b);
	s = (robot_status_t){ VC_FAULT, -20, 0.04f, FAULT_IMPACT, 0 };
	tm_format_status(b, sizeof b, &s);
	CHECK(strstr(b, "\"speed\":-20") && strstr(b, "\"max_hump_mm\":0.0") && strstr(b, "IMPACT") && strstr(b, "FAULT"), "fault JSON: %s", b);
	CHECK(tm_format_status(b, 20, &s) == -1, "too small buffer -> -1");
	CHECK(tm_format_heartbeat(b, sizeof b, 7, 4000) > 0 && !strcmp(b, "{\"hb\":7,\"t\":4000}"), "heartbeat: %s", b);
	CHECK(tm_parse_cmd("start", 5) == REMOTE_START, "start");
	CHECK(tm_parse_cmd("STOP", 4) == REMOTE_STOP, "STOP");
	CHECK(tm_parse_cmd("{\"cmd\":\"Reset\"}", 15) == REMOTE_RESET, "json reset");
	CHECK(tm_parse_cmd("go", 2) == -1, "unknown ignored");
	CHECK(tm_parse_cmd("start stop", 10) == -1, "ambiguous ignored");
	CHECK(tm_parse_cmd("", 0) == -1 && tm_parse_cmd(0, 5) == -1, "empty ignored");
	CHECK(!strcmp(tm_state_name(99), "UNKNOWN") && !strcmp(tm_fault_name(-1), "UNKNOWN"), "out of range names");
	/* status names match the VC's enum order */
	for (int i = VC_INIT; i <= VC_FAULT; i++) CHECK(strcmp(tm_state_name(i), "UNKNOWN") != 0, "state %d named", i);
	printf("%d checks, %d failed\n", checks, fails);
	printf(fails ? "SOME FAILED\n" : "ALL PASSED\n");
	return fails != 0;
}
