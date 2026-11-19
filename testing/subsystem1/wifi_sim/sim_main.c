/*
 * sim_main.c -- WiFi/MQTT scenarios for Subsystem 1. Runs the real lib/liblwip/.../mqtt_utk.c, app_program/subsystem1/
 * subsystem1.c and telemetry.c on a PC, on top of fake_net.c. One scenario per process: ./wifi_sim <name> [arg].
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"
#include "../../../app_program/subsystem5/vc_msgs.h"

extern volatile char s3_last_barcode[8];
extern volatile char s3_last_pattern[80];
extern volatile int s3_last_nav;
extern volatile unsigned s3_barcode_count;

static int checks, fails;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
			   else { printf("  ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t next_status, last_connect_t; static int seen_connects;
static int burst_every_ms;
static void run_ms(uint32_t ms)
{
	uint32_t end = SIM.now + ms;
	while (SIM.now < end && !SIM.halted) {
		sim_poll();
		if (SIM.connects_ok != seen_connects) { seen_connects = SIM.connects_ok; last_connect_t = SIM.now; }
		if ((int32_t)(SIM.now - next_status) >= 0) { next_status += 500; sim_vc_status(VC_LINE_FOLLOW, 150, 12.3f, FAULT_NONE); }
		if (burst_every_ms && SIM.now % burst_every_ms == 0) {	/* a state change burst + a barcode read in the same 20 ms */
			sim_vc_status(VC_BARCODE_ACTION, 0, 12.3f, FAULT_NONE);
			sim_vc_status(VC_LINE_FOLLOW, 80, 12.3f, FAULT_NONE);
			sim_vc_status(VC_LINE_FOLLOW, 150, 12.3f, FAULT_NONE);
			s3_last_barcode[0] = 'B'; s3_last_barcode[1] = 0; s3_last_nav = 1;
			strcpy((char *)s3_last_pattern, "100101101101001010110110100101101101");
			s3_barcode_count++;
		}
	}
}
static void report(void)
{
	printf("  -- t=%.1f s  connects=%d  connect() calls=%d  ERR_ISCONN=%d  'connect() failed' console lines=%d  "
	       "timer pool peak=%d/%d%s\n", SIM.now / 1000.0, SIM.connects_ok, SIM.connect_calls, SIM.isconn_errs,
	       SIM.console_connect_failed, SIM.pool_peak, SIM.pool_cap, SIM.halted ? "  ** HALTED **" : "");
	for (int i = 0; i < 5; i++)
		printf("     %-24s queued=%-4d delivered=%-4d ERR_MEM(req slots)=%-3d ERR_MEM(ring)=%-3d ERR_CONN=%d\n", SIM.t[i].name,
		       SIM.t[i].pub_ok, SIM.t[i].delivered, SIM.t[i].pub_err_req, SIM.t[i].pub_err_ring, SIM.t[i].pub_err_conn);
	printf("     heartbeats missing=%d  max heartbeat gap=%u ms  oldest status delivered=%u ms old  VC status sent=%d dropped(mbf full)=%d\n",
	       SIM.hb_gaps, SIM.hb_max_gap_ms, SIM.status_max_age_ms, SIM.vc_sent, SIM.vc_dropped);
	if (SIM.halted) printf("     %s\n", SIM.halt_msg);
}

static void sc_connect(void)
{
	printf("[connect] broker up, RTT 20 ms, 60 s of normal running\n");
	run_ms(60000);
	report();
	CHECK(SIM.connects_ok == 1, "connected once");
	CHECK(sim_inject("robot/se18/cmd", "noop", 0), "robot/se18/cmd is subscribed at the broker");
	CHECK(SIM.t[0].delivered >= 118 && SIM.t[0].delivered <= 121, "status at 2 Hz (%d in 60 s)", SIM.t[0].delivered);
	CHECK(SIM.t[1].delivered >= 58 && SIM.t[1].delivered <= 61, "heartbeat at 1 Hz (%d in 60 s)", SIM.t[1].delivered);
	CHECK(SIM.hb_gaps == 0 && SIM.hb_max_gap_ms <= 1150, "no heartbeat missing, max gap %u ms", SIM.hb_max_gap_ms);
	CHECK(!strncmp(SIM.t[0].last, "{\"state\":\"LINE_FOLLOW\",\"speed\":150,\"max_hump_mm\":12.3,\"fault\":\"NONE\",\"t\":", 72),
	      "status JSON: %s", SIM.t[0].last);
	CHECK(SIM.t[0].pub_err_req + SIM.t[0].pub_err_ring + SIM.t[1].pub_err_req + SIM.t[1].pub_err_ring == 0, "no publish refused");
	CHECK(SIM.t[4].delivered >= 11, "template heartbeat still published every 5 s (%d)", SIM.t[4].delivered);
	CHECK(SIM.status_max_age_ms <= 200, "status latency %u ms", SIM.status_max_age_ms);
}

static void sc_commands(void)
{
	printf("[commands] remote commands on robot/se18/cmd -> mbf_remote_cmd (what vc_task polls)\n");
	run_ms(2000);
	struct { const char *topic, *payload; int frag, expect; } T[] = {
		{ "robot/se18/cmd", "start", 0, REMOTE_START },
		{ "robot/se18/cmd", "{\"cmd\":\"STOP\"}", 0, REMOTE_STOP },
		{ "robot/se18/cmd", "Reset", 0, REMOTE_RESET },
		{ "robot/se18/cmd", "st" "art", 2, REMOTE_START },		/* arrives in 2-byte fragments */
		{ "robot/se18/cmd", "restart", 0, -1 },
		{ "robot/se18/cmd", "start stop", 0, -1 },
		{ "robot/se18/cmd", "go", 0, -1 },
		{ "robot/se18/cmd", "\x01\x02stop\xff", 0, REMOTE_STOP },	/* control bytes become '.' */
	};
	for (unsigned i = 0; i < sizeof T / sizeof T[0]; i++) {
		int routed = sim_inject(T[i].topic, T[i].payload, T[i].frag);
		uint32_t t0 = SIM.now; run_ms(30);
		int got = sim_remote_cmd_take();
		CHECK(routed && got == T[i].expect, "\"%s\"%s -> %d (expected %d) within %u ms", T[i].payload,
		      T[i].frag ? " (fragmented)" : "", got, T[i].expect, SIM.now - t0);
	}
	char big[220]; memset(big, 'x', sizeof big); memcpy(big + 200, " stop", 6);
	sim_inject("robot/se18/cmd", big, 0); run_ms(30);
	CHECK(sim_remote_cmd_take() == -1, "205-byte payload with 'stop' after byte 127 is truncated and ignored");
	memcpy(big, "stop ", 5); big[205] = 0; sim_inject("robot/se18/cmd", big, 0); run_ms(30);
	CHECK(sim_remote_cmd_take() == REMOTE_STOP, "205-byte payload starting with 'stop' still works (truncated to 127)");
	CHECK(!sim_inject("robot/se18/cmdx", "stop", 0), "a different topic is not delivered");
	for (int i = 0; i < 12; i++) sim_inject("robot/se18/cmd", "stop", 0);
	run_ms(50);
	int n = 0; while (sim_remote_cmd_take() >= 0) n++;
	CHECK(n == 8, "12 commands while the VC is busy: %d kept (mbf_remote_cmd depth), rest dropped without blocking", n);
}

static void sc_barcode(void)
{
	printf("[barcode] barcode reads -> robot/se18/barcode\n");
	run_ms(2000);
	strcpy((char *)s3_last_barcode, "A"); strcpy((char *)s3_last_pattern, "1010"); s3_last_nav = 0; s3_barcode_count = 1;
	uint32_t t0 = SIM.now; run_ms(300);
	CHECK(SIM.t[2].delivered == 1, "one publish per read (%d)", SIM.t[2].delivered);
	char exp[120]; snprintf(exp, sizeof exp, "{\"code\":\"A\",\"cmd\":\"LEFT\",\"n\":1,\"pattern\":\"1010\",\"t\":");
	CHECK(!strncmp(SIM.t[2].last, exp, strlen(exp)), "barcode JSON: %s (sent ~%u ms after the read)", SIM.t[2].last,
	      (unsigned)(strtoul(strstr(SIM.t[2].last, "\"t\":") + 4, 0, 10) - t0));
	s3_last_barcode[0] = 0; s3_last_nav = -1; s3_barcode_count = 2; run_ms(300);
	CHECK(strstr(SIM.t[2].last, "\"cmd\":\"UNREADABLE\"") != NULL, "failed read: %s", SIM.t[2].last);
	s3_barcode_count = 3; s3_barcode_count = 4; run_ms(300);
	CHECK(SIM.t[2].delivered == 3 && strstr(SIM.t[2].last, "\"n\":4"), "two reads within 100 ms: only the newest is sent (n=4), count %d", SIM.t[2].delivered);
}

static void sc_rtt(int rtt)
{
	printf("[rtt %d] 60 s, VC 2 Hz + every 5 s a state-change burst (3 statuses) and a barcode read\n", rtt);
	SIM.rtt_ms = rtt; burst_every_ms = 5000;
	run_ms(62000);
	report();
	int lost_status = SIM.vc_sent - SIM.vc_dropped - SIM.t[0].delivered - 1;	/* at most 1 still in flight at the end */
	CHECK(lost_status <= 0, "every status the VC queued reached the broker (%d of %d; lwIP refused a publish %d times)",
	      SIM.t[0].delivered, SIM.vc_sent - SIM.vc_dropped, SIM.t[0].pub_err_req + SIM.t[0].pub_err_ring);
	CHECK(SIM.hb_gaps == 0, "no heartbeat lost (%d)", SIM.hb_gaps);
	CHECK(SIM.t[2].delivered >= 11, "every barcode read reached the broker (%d of 12)", SIM.t[2].delivered);
}

static void sc_outage(int mode, uint32_t down_at, uint32_t down_for, uint32_t total)
{
	printf("[outage %s] broker %s at %u s for %u s\n", mode == BROKER_CLOSED ? "closed" : "silent",
	       mode == BROKER_CLOSED ? "process stopped (port refuses)" : "unreachable (laptop left the hotspot)",
	       down_at / 1000, down_for / 1000);
	run_ms(down_at - 1000 + 1000);
	int c0 = SIM.connects_ok;
	sim_set_broker(mode);
	int st_before = SIM.t[0].delivered;
	run_ms(down_for);
	int during = SIM.t[0].delivered - st_before;
	sim_set_broker(BROKER_UP);
	uint32_t up_t = SIM.now;
	run_ms(total - down_at - down_for);
	report();
	CHECK(!SIM.halted, "the WiFi task did not halt");
	CHECK(SIM.connects_ok > c0 && last_connect_t >= up_t, "reconnected %.1f s after the broker came back",
	      (last_connect_t - up_t) / 1000.0);
	CHECK(SIM.console_connect_failed < 20, "console not flooded ('connect() failed' printed %d times)", SIM.console_connect_failed);
	printf("     statuses delivered while broker was down: %d\n", during);
}

int main(int argc, char **argv)
{
	const char *s = argc > 1 ? argv[1] : "connect";
	sim_reset();
	SIM.verbose = getenv("VERBOSE") != NULL;
	if (getenv("RTT")) SIM.rtt_ms = atoi(getenv("RTT"));
	next_status = SIM.now + 500;
	if (!strcmp(s, "connect")) sc_connect();
	else if (!strcmp(s, "commands")) sc_commands();
	else if (!strcmp(s, "barcode")) sc_barcode();
	else if (!strcmp(s, "rtt")) sc_rtt(argc > 2 ? atoi(argv[2]) : 100);
	else if (!strcmp(s, "closed")) sc_outage(BROKER_CLOSED, 20000, argc > 2 ? atoi(argv[2]) * 1000 : 30000, (argc > 2 ? atoi(argv[2]) * 1000 : 30000) + 60000);
	else if (!strcmp(s, "silent")) sc_outage(BROKER_SILENT, 20000, argc > 2 ? atoi(argv[2]) * 1000 : 180000, (argc > 2 ? atoi(argv[2]) * 1000 : 180000) + 120000);
	else { printf("unknown scenario %s\n", s); return 2; }
	printf("  => %d checks, %d failed\n\n", checks, fails);
	return fails != 0;
}
