/*
 * fake_net.c -- PC model of what mqtt_utk.c / subsystem1.c sit on: lwIP's MQTT client (apps/mqtt/mqtt.c, lwIP 2.x
 * defaults MQTT_REQ_MAX_IN_FLIGHT 4, MQTT_OUTPUT_RINGBUF_SIZE 768 (SE18 lwipopts.h), MQTT_CYCLIC_TIMER_INTERVAL 5 s, MQTT_REQ_TIMEOUT 30 s,
 * keep-alive watchdog 1.5 x keep_alive, no automatic reconnect), lwIP's sys_timeout pool, TCP to a broker with a given
 * round-trip time, a broker, and micro T-Kernel message buffers. Behaviour copied from lwIP, not lwIP's code.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "tm/tmonitor.h"
#include "tk/tkernel.h"
#include "sim.h"
#include "../../../app_program/subsystem5/vc_msgs.h"

sim_t SIM;
static const char *const topic_names[NTOPIC] = { "robot/se18/status", "robot/se18/heartbeat", "robot/se18/barcode",
						 "robot/se18/cmd", "mtk3smp-pico/heartbeat", "other" };
int sim_topic(const char *t) { for (int i = 0; i < NTOPIC - 1; i++) if (!strcmp(t, topic_names[i])) return i; return NTOPIC - 1; }

/* ---------------- console ---------------- */
int tm_printf(const UB *fmt, ...)
{
	char b[300]; va_list ap; va_start(ap, fmt); int n = vsnprintf(b, sizeof b, (const char *)fmt, ap); va_end(ap);
	SIM.console_lines++;
	if (strstr(b, "mqtt_client_connect() failed")) SIM.console_connect_failed++;
	if (SIM.verbose) printf("    [%7.3f] %s", SIM.now / 1000.0, b);
	return n;
}
int tm_sprintf(UB *buf, const UB *fmt, ...)
{ va_list ap; va_start(ap, fmt); int n = vsprintf((char *)buf, (const char *)fmt, ap); va_end(ap); return n; }

/* ---------------- sys_timeout pool ---------------- */
typedef struct { int used; uint32_t due, seq; sys_timeout_handler fn; void *arg; } tmr_t;
static tmr_t tm[32]; static uint32_t tseq;
static int pool_used(void) { int n = 0; for (int i = 0; i < 32; i++) n += tm[i].used; return n; }
u32_t sys_now(void) { return SIM.now; }
void sys_timeout(u32_t ms, sys_timeout_handler fn, void *arg)
{
	if (pool_used() >= SIM.pool_cap) {		/* memp_malloc(MEMP_SYS_TIMEOUT) == NULL -> LWIP_ASSERT -> halt */
		if (!SIM.halted) {
			SIM.halted = 1;
			snprintf(SIM.halt_msg, sizeof SIM.halt_msg, "lwIP assert: sys_timeout: pool MEMP_SYS_TIMEOUT is empty (t=%.1f s)", SIM.now / 1000.0);
		}
		return;
	}
	for (int i = 0; i < 32; i++) if (!tm[i].used) { tm[i] = (tmr_t){ 1, SIM.now + ms, tseq++, fn, arg }; break; }
	int u = pool_used(); if (u > SIM.pool_peak) SIM.pool_peak = u;
}
void sys_untimeout(sys_timeout_handler fn, void *arg)	/* lwIP removes the first (soonest) match only */
{
	int best = -1;
	for (int i = 0; i < 32; i++) if (tm[i].used && tm[i].fn == fn && tm[i].arg == arg && (best < 0 || tm[i].due < tm[best].due)) best = i;
	if (best >= 0) tm[best].used = 0;
}
static void run_timers(void)
{
	for (;;) {
		int best = -1;
		for (int i = 0; i < 32; i++) if (tm[i].used && (int32_t)(tm[i].due - SIM.now) <= 0 &&
						(best < 0 || tm[i].due < tm[best].due || (tm[i].due == tm[best].due && tm[i].seq < tm[best].seq))) best = i;
		if (best < 0 || SIM.halted) return;
		tmr_t t = tm[best]; tm[best].used = 0;	/* slot is freed before the handler runs, as in lwIP */
		t.fn(t.arg);
	}
}

/* ---------------- micro T-Kernel message buffers ---------------- */
typedef struct { int used, cap, maxmsz, head, n; int sz[64]; unsigned char m[64][64]; } mbf_t;
static mbf_t mbf[8];
ID tk_cre_mbf(const T_CMBF *pk)
{
	for (int i = 1; i < 8; i++) if (!mbf[i].used) {
		mbf[i] = (mbf_t){ .used = 1, .maxmsz = pk->maxmsz, .cap = pk->bufsz / (((pk->maxmsz + 3) & ~3) + 4) };
		if (mbf[i].cap > 64) mbf[i].cap = 64;
		return i;
	}
	return -34;
}
ER tk_snd_mbf(ID id, const void *msg, INT sz, TMO tmo)
{
	(void)tmo; mbf_t *b = &mbf[id];
	if (id <= 0 || !b->used || sz > b->maxmsz) return -17;
	if (b->n >= b->cap) return E_TMOUT;
	int k = (b->head + b->n++) % 64; memcpy(b->m[k], msg, sz); b->sz[k] = sz; return E_OK;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo)
{
	(void)tmo; mbf_t *b = &mbf[id];
	if (id <= 0 || !b->used) return -17;
	if (!b->n) return E_TMOUT;
	int k = b->head; b->head = (b->head + 1) % 64; b->n--; memcpy(msg, b->m[k], b->sz[k]); return b->sz[k];
}

/* ---------------- objects other subsystems would provide ---------------- */
ID mbf_remote_cmd;				/* S5's buffer: overrides subsystem1.c's weak 0 */
volatile char s3_last_barcode[8];
volatile char s3_last_pattern[80];
volatile int s3_last_nav;
volatile unsigned s3_barcode_count;
volatile unsigned s3_barcode_seq;
volatile obstacle_profile_t s5_profile;
volatile unsigned s5_profile_seq;
extern ID mbf_status;
extern int subsystem1_init(void);

void sim_vc_status(int state, int speed, float hump, int fault)
{
	robot_status_t st = { state, speed, hump, fault, SIM.now };
	SIM.vc_sent++;
	if (tk_snd_mbf(mbf_status, &st, sizeof st, TMO_POL) != E_OK) SIM.vc_dropped++;
}
int sim_remote_cmd_take(void) { remote_cmd_t m; return tk_rcv_mbf(mbf_remote_cmd, &m, TMO_POL) == (INT)sizeof m ? m.cmd : -1; }

/* ---------------- lwIP MQTT client + TCP + broker ---------------- */
enum { TCP_DISCONNECTED, TCP_CONNECTING, MQTT_CONNECTING, MQTT_CONNECTED };
struct mqtt_client_s {
	int state, epoch; mqtt_connection_cb_t cb; void *cb_arg;
	mqtt_incoming_publish_cb_t pub_cb; mqtt_incoming_data_cb_t data_cb; void *in_arg;
	struct { int used; u16_t id; uint32_t t; mqtt_request_cb_t cb; void *arg; char topic[64]; } req[4];
	u16_t next_id; u16_t keep_alive; uint32_t last_rx, t_connect;
	int ring, unacked;			/* bytes in the 768-byte output ring / sent but not ACKed (TCP_SND_BUF 11680) */
	char subs[4][64]; int nsubs;		/* broker side: this client's subscriptions (clean session) */
	int syn_idx;
};
static mqtt_client_t *the_client;

enum { EV_TCP_UP, EV_SYN_RETRY, EV_TCP_FAIL, EV_CONNACK, EV_ACK, EV_SUBACK, EV_PEER_CLOSE, EV_DELIVER, EV_INBOUND };
typedef struct { int used; uint32_t due, seq; int type, epoch, a; char topic[64]; char data[768]; int len, frag; } ev_t;
static ev_t ev[512]; static uint32_t eseq;
static void post(uint32_t delay, int type, int epoch, int a, const char *topic, const void *data, int len, int frag)
{
	for (int i = 0; i < 512; i++) if (!ev[i].used) {
		ev[i] = (ev_t){ 1, SIM.now + delay, eseq++, type, epoch, a, "", "", len, frag };
		if (topic) snprintf(ev[i].topic, sizeof ev[i].topic, "%s", topic);
		if (data) memcpy(ev[i].data, data, len > 511 ? 511 : len);
		return;
	}
	fprintf(stderr, "event queue full\n"); exit(2);
}
static const uint32_t syn_at[] = { 3000, 9000, 18000, 30000, 45000, 63000 };	/* lwIP SYN back-off, RTO 3 s, SYNMAXRTX 6 */

static void mqtt_cyclic_timer(void *arg);
static void mqtt_close(mqtt_client_t *c, mqtt_connection_status_t why)
{
	c->state = TCP_DISCONNECTED; c->epoch++; c->nsubs = 0; c->ring = c->unacked = 0;
	memset(c->req, 0, sizeof c->req);
	sys_untimeout(mqtt_cyclic_timer, c);
	if (c->cb) c->cb(c, c->cb_arg, why);
}
static void mqtt_cyclic_timer(void *arg)
{
	mqtt_client_t *c = arg;
	for (int i = 0; i < 4; i++)				/* MQTT_REQ_TIMEOUT */
		if (c->req[i].used && SIM.now - c->req[i].t >= 30000) { c->req[i].used = 0; if (c->req[i].cb) c->req[i].cb(c->req[i].arg, ERR_TIMEOUT); }
	if (c->state == MQTT_CONNECTING && SIM.now - c->t_connect >= 100000) { mqtt_close(c, MQTT_CONNECT_TIMEOUT); return; }
	if (c->state == MQTT_CONNECTED) {
		if (c->keep_alive && SIM.now - c->last_rx >= c->keep_alive * 1500u) { mqtt_close(c, MQTT_CONNECT_TIMEOUT); return; }
		if (SIM.broker == BROKER_UP) post(SIM.rtt_ms, EV_ACK, c->epoch, 0, 0, 0, 0, 0);	/* PINGREQ/PINGRESP */
	}
	sys_timeout(5000, mqtt_cyclic_timer, c);
}
mqtt_client_t *mqtt_client_new(void) { return the_client = calloc(1, sizeof *the_client); }
u8_t mqtt_client_is_connected(mqtt_client_t *c) { return c && c->state == MQTT_CONNECTED; }
void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t p, mqtt_incoming_data_cb_t d, void *a)
{ c->pub_cb = p; c->data_cb = d; c->in_arg = a; }
int ip4addr_aton(const char *cp, ip_addr_t *a)
{ unsigned x[4]; if (sscanf(cp, "%u.%u.%u.%u", &x[0], &x[1], &x[2], &x[3]) != 4) return 0; a->addr = x[0] | x[1] << 8 | x[2] << 16 | x[3] << 24; return 1; }

err_t mqtt_client_connect(mqtt_client_t *c, const void *ip, u16_t port, mqtt_connection_cb_t cb, void *arg,
			  const struct mqtt_connect_client_info_t *ci)
{
	(void)ip; (void)port;
	SIM.connect_calls++;
	if (c->state != TCP_DISCONNECTED) { SIM.isconn_errs++; return ERR_ISCONN; }
	int ep = c->epoch + 1;
	memset(c, 0, sizeof *c);		/* "Wipe clean", as lwIP does: the inpub callbacks are gone too */
	c->epoch = ep; c->cb = cb; c->cb_arg = arg; c->keep_alive = ci->keep_alive; c->state = TCP_CONNECTING; c->t_connect = SIM.now;
	if (SIM.broker == BROKER_UP) post(SIM.rtt_ms, EV_TCP_UP, c->epoch, 0, 0, 0, 0, 0);
	else if (SIM.broker == BROKER_CLOSED) post(SIM.rtt_ms, EV_TCP_FAIL, c->epoch, 0, 0, 0, 0, 0);
	else post(syn_at[0], EV_SYN_RETRY, c->epoch, 0, 0, 0, 0, 0);
	return ERR_OK;
}
static int out_space(mqtt_client_t *c, int remaining)
{
	int total = 1 + (remaining > 127 ? 2 : 1) + remaining;
	if (c->ring + total > 768) return 0;
	c->ring += total;
	int room = 11680 - c->unacked, mv = c->ring < room ? c->ring : room;	/* mqtt_output_send: tcp_write(COPY) */
	c->ring -= mv; c->unacked += mv;
	return 1;
}
static int req_alloc(mqtt_client_t *c, u16_t id, mqtt_request_cb_t cb, void *arg)
{
	for (int i = 0; i < 4; i++) if (!c->req[i].used) { c->req[i].used = 1; c->req[i].id = id; c->req[i].t = SIM.now; c->req[i].cb = cb; c->req[i].arg = arg; return i; }
	return -1;
}
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload, u16_t len, u8_t qos, u8_t retain,
		   mqtt_request_cb_t cb, void *arg)
{
	(void)qos; (void)retain;
	int ti = sim_topic(topic);
	if (c->state == TCP_DISCONNECTED) { SIM.t[ti].pub_err_conn++; return ERR_CONN; }
	int r = req_alloc(c, 0, cb, arg);
	if (r < 0) { SIM.t[ti].pub_err_req++; return ERR_MEM; }
	if (!out_space(c, 2 + (int)strlen(topic) + len)) { c->req[r].used = 0; SIM.t[ti].pub_err_ring++; return ERR_MEM; }
	SIM.t[ti].pub_ok++;
	if (SIM.broker == BROKER_UP) {
		post(SIM.rtt_ms / 2, EV_DELIVER, c->epoch, 0, topic, payload, len, 0);
		post(SIM.rtt_ms + SIM.ack_delay_ms, EV_ACK, c->epoch, 0, 0, 0, 0, 0);
	}
	return ERR_OK;
}
err_t mqtt_sub_unsub(mqtt_client_t *c, const char *topic, u8_t qos, mqtt_request_cb_t cb, void *arg, u8_t sub)
{
	(void)qos; (void)sub;
	if (c->state == TCP_DISCONNECTED) return ERR_CONN;
	u16_t id = ++c->next_id ? c->next_id : ++c->next_id;
	int r = req_alloc(c, id, cb, arg);
	if (r < 0) return ERR_MEM;
	if (!out_space(c, 2 + 2 + (int)strlen(topic) + 1)) { c->req[r].used = 0; return ERR_MEM; }
	snprintf(c->req[r].topic, 64, "%s", topic);
	if (SIM.broker == BROKER_UP) post(SIM.rtt_ms, EV_SUBACK, c->epoch, id, 0, 0, 0, 0);
	return ERR_OK;
}

static void deliver(ev_t *e)
{
	int ti = sim_topic(e->topic);
	SIM.t[ti].delivered++;
	e->data[e->len < (int)sizeof e->data - 1 ? e->len : (int)sizeof e->data - 1] = 0;
	snprintf(SIM.t[ti].last, sizeof SIM.t[ti].last, "%s", e->data);
	if (ti == 1) {				/* robot/se18/heartbeat: {"hb":N,"t":T} */
		int seq = 0; sscanf(e->data, "{\"hb\":%d", &seq);
		if (SIM.hb_last_seq && seq != SIM.hb_last_seq + 1) SIM.hb_gaps += seq - SIM.hb_last_seq - 1;
		if (SIM.hb_last_t && SIM.now - SIM.hb_last_t > SIM.hb_max_gap_ms) SIM.hb_max_gap_ms = SIM.now - SIM.hb_last_t;
		SIM.hb_last_seq = seq; SIM.hb_last_t = SIM.now;
	}
	if (ti == 0) {
		const char *p = strstr(e->data, "\"t\":"); unsigned ts = p ? (unsigned)strtoul(p + 4, 0, 10) : 0;
		if (SIM.now - ts > SIM.status_max_age_ms) SIM.status_max_age_ms = SIM.now - ts;
	}
	mqtt_client_t *c = the_client;		/* broker forwards to subscribers */
	for (int i = 0; c && i < c->nsubs; i++) if (!strcmp(c->subs[i], e->topic))
		post(SIM.rtt_ms / 2, EV_INBOUND, c->epoch, 0, e->topic, e->data, e->len, 0);
}
static void inbound(mqtt_client_t *c, ev_t *e)
{
	if (c->state != MQTT_CONNECTED || !c->pub_cb) return;
	c->last_rx = SIM.now;
	c->pub_cb(c->in_arg, e->topic, (u32_t)e->len);
	int f = e->frag > 0 ? e->frag : (e->len ? e->len : 1);
	for (int off = 0; off < e->len || off == 0; off += f) {
		int n = e->len - off < f ? e->len - off : f;
		c->data_cb(c->in_arg, (const u8_t *)e->data + off, (u16_t)n, off + n >= e->len ? MQTT_DATA_FLAG_LAST : 0);
		if (e->len == 0) break;
	}
}
int sim_inject(const char *topic, const char *payload, int frag)
{
	mqtt_client_t *c = the_client;
	if (!c || c->state != MQTT_CONNECTED || SIM.broker != BROKER_UP) return 0;
	for (int i = 0; i < c->nsubs; i++) if (!strcmp(c->subs[i], topic)) {
		post(SIM.rtt_ms / 2, EV_INBOUND, c->epoch, 0, topic, payload, (int)strlen(payload), frag); return 1;
	}
	return 0;
}
static void run_net(void)
{
	for (;;) {
		int b = -1;
		for (int i = 0; i < 512; i++) if (ev[i].used && (int32_t)(ev[i].due - SIM.now) <= 0 &&
						(b < 0 || ev[i].due < ev[b].due || (ev[i].due == ev[b].due && ev[i].seq < ev[b].seq))) b = i;
		if (b < 0 || SIM.halted) return;
		ev_t e = ev[b]; ev[b].used = 0;
		mqtt_client_t *c = the_client;
		if (!c || e.epoch != c->epoch) continue;		/* belongs to a closed connection */
		switch (e.type) {
		case EV_SYN_RETRY:
			if (c->state != TCP_CONNECTING) break;
			if (SIM.broker == BROKER_UP) post(SIM.rtt_ms, EV_TCP_UP, c->epoch, 0, 0, 0, 0, 0);
			else if (SIM.broker == BROKER_CLOSED) post(SIM.rtt_ms, EV_TCP_FAIL, c->epoch, 0, 0, 0, 0, 0);
			else if (++c->syn_idx < 6) post(syn_at[c->syn_idx] - syn_at[c->syn_idx - 1], EV_SYN_RETRY, c->epoch, 0, 0, 0, 0, 0);
			else post(0, EV_TCP_FAIL, c->epoch, 0, 0, 0, 0, 0);
			break;
		case EV_TCP_UP:
			if (SIM.broker == BROKER_CLOSED) { SIM.tcp_fails++; mqtt_close(c, MQTT_CONNECT_DISCONNECTED); break; }
			c->state = MQTT_CONNECTING; sys_timeout(5000, mqtt_cyclic_timer, c);
			if (SIM.broker == BROKER_UP) post(SIM.rtt_ms, EV_CONNACK, c->epoch, 0, 0, 0, 0, 0);
			break;
		case EV_TCP_FAIL: SIM.tcp_fails++; mqtt_close(c, MQTT_CONNECT_DISCONNECTED); break;
		case EV_PEER_CLOSE: mqtt_close(c, MQTT_CONNECT_DISCONNECTED); break;
		case EV_CONNACK:
			c->state = MQTT_CONNECTED; c->last_rx = SIM.now; SIM.connects_ok++;
			if (c->cb) c->cb(c, c->cb_arg, MQTT_CONNECT_ACCEPTED);
			break;
		case EV_ACK:				/* mqtt_tcp_sent_cb: every QoS 0 request is complete */
			c->last_rx = SIM.now; c->unacked = 0;
			for (int i = 0; i < 4; i++) if (c->req[i].used && c->req[i].id == 0) { c->req[i].used = 0; if (c->req[i].cb) c->req[i].cb(c->req[i].arg, ERR_OK); }
			{ int mv = c->ring; c->ring = 0; c->unacked += mv; }
			break;
		case EV_SUBACK:
			c->last_rx = SIM.now;
			for (int i = 0; i < 4; i++) if (c->req[i].used && c->req[i].id == e.a) {
				if (c->nsubs < 4) snprintf(c->subs[c->nsubs++], 64, "%s", c->req[i].topic);
				c->req[i].used = 0; if (c->req[i].cb) c->req[i].cb(c->req[i].arg, ERR_OK);
			}
			break;
		case EV_DELIVER: deliver(&e); break;
		case EV_INBOUND: inbound(c, &e); break;
		}
	}
}
void sim_set_broker(int mode)
{
	mqtt_client_t *c = the_client;
	if (c && c->state != TCP_DISCONNECTED && SIM.broker == BROKER_UP && mode == BROKER_CLOSED)
		post(SIM.rtt_ms / 2, EV_PEER_CLOSE, c->epoch, 0, 0, 0, 0, 0);	/* broker process exits: FIN/RST */
	SIM.broker = mode;
}
int sim_client_state(void) { return the_client ? the_client->state : -1; }

/* one iteration of cyw43_utk_task(): cyw43_poll (network in), sys_check_timeouts(), mqtt_utk_start() if link up */
void sim_poll(void)
{
	if (SIM.halted) return;
	SIM.now += 2;
	run_net();
	run_timers();
	if (SIM.link_up && !SIM.halted) mqtt_utk_start();
}

void sim_reset(void)
{
	memset(&SIM, 0, sizeof SIM); memset(tm, 0, sizeof tm); memset(ev, 0, sizeof ev); memset(mbf, 0, sizeof mbf);
	for (int i = 0; i < NTOPIC; i++) SIM.t[i].name = topic_names[i];
	SIM.rtt_ms = 20; SIM.pool_cap = 4; SIM.link_up = 1; SIM.broker = BROKER_UP; SIM.now = 1000;
	free(the_client); the_client = NULL;
	T_CMBF ck = { 0, TA_TFIFO, 8 * (sizeof(remote_cmd_t) + 4), sizeof(remote_cmd_t) };
	mbf_remote_cmd = tk_cre_mbf(&ck);
	subsystem1_init();
	s3_barcode_count = 0;
}
