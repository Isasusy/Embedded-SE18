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
	CHECK(tm_parse_cmd("restart", 7) == -1, "part of a word ignored");
	CHECK(tm_parse_cmd("{\"cmd\":\"STOP\"}", 14) == REMOTE_STOP, "json stop");
	CHECK(tm_parse_cmd("", 0) == -1 && tm_parse_cmd(0, 5) == -1, "empty ignored");
	CHECK(!strcmp(tm_state_name(99), "UNKNOWN") && !strcmp(tm_fault_name(-1), "UNKNOWN"), "out of range names");
	/* status names match the VC's enum order */
	for (int i = VC_INIT; i <= VC_FAULT; i++) CHECK(strcmp(tm_state_name(i), "UNKNOWN") != 0, "state %d named", i);
	/* obstacle profile JSON */
	{
		char ob[512];
		obstacle_profile_t p = { 5, { 30, 60, 90, 120, 150 }, { -1, 412, 198, 230, -1 },
					 { true, 198.4f, 120.6f, 300.2f, 89.5f, +1 }, 98765 };
		int k = tm_format_obstacle(ob, sizeof ob, &p, 3);
		CHECK(k == (int)strlen(ob), "obstacle length matches");
		CHECK(!strcmp(ob, "{\"n\":3,\"deg\":[30,60,90,120,150],\"mm\":[-1,412,198,230,-1],\"found\":true,"
			"\"dist_mm\":198,\"width_mm\":121,\"left_clear_mm\":300,\"right_clear_mm\":90,\"side\":\"LEFT\",\"t\":98765}"),
		      "obstacle JSON: %s", ob);
		p.ob.side = 0; p.ob.found = false;
		tm_format_obstacle(ob, sizeof ob, &p, 4);
		CHECK(strstr(ob, "\"found\":false") && strstr(ob, "\"side\":\"NONE\""), "no obstacle JSON: %s", ob);
		CHECK(tm_format_obstacle(ob, 40, &p, 4) == -1, "obstacle: too small buffer -> -1");
		/* worst case still fits S1's 512-byte buffer, with the topic and MQTT header, in lwIP's 512-byte MQTT ring */
		obstacle_profile_t w = { OBST_PROFILE_MAX_RAYS, {0}, {0}, { true, -99999, -99999, -99999, -99999, -1 }, 4294967295u };
		for (int i = 0; i < OBST_PROFILE_MAX_RAYS; i++) { w.servo_deg[i] = 150; w.dist_mm[i] = 9999; }
		k = tm_format_obstacle(ob, sizeof ob, &w, 4294967295u);
		CHECK(k > 0 && k + 5 + 2 + (int)strlen(S1_TOPIC_OBSTACLE) <= 512, "worst-case obstacle JSON is %d bytes", k);
	}
	printf("%d checks, %d failed\n", checks, fails);
	printf(fails ? "SOME FAILED\n" : "ALL PASSED\n");
	return fails != 0;
}
