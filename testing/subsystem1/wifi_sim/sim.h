/* Shared state of the WiFi/MQTT PC harness. See README.md. */
#ifndef SIM_H
#define SIM_H
#include <stdint.h>
enum { BROKER_UP, BROKER_CLOSED, BROKER_SILENT };	/* closed = host up, port refuses (RST); silent = host gone */

#define NTOPIC 6
typedef struct {
	/* configuration */
	uint32_t rtt_ms, ack_delay_ms;
	int broker;
	int pool_cap;				/* free MEMP_SYS_TIMEOUT slots above lwIP's own (lwipopts.h: +4) */
	int link_up;
	int verbose;
	/* state */
	uint32_t now;
	int halted; char halt_msg[120];
	/* statistics */
	int pool_peak;
	int connects_ok, connect_calls, isconn_errs, tcp_fails;
	int console_lines, console_connect_failed;
	struct { const char *name; int pub_ok, pub_err_req, pub_err_ring, pub_err_conn, delivered; char last[512]; } t[NTOPIC];
	int hb_last_seq, hb_gaps; uint32_t hb_last_t, hb_max_gap_ms;	/* delivered robot/se18/heartbeat */
	uint32_t status_max_age_ms;					/* deliver time - status timestamp */
	int vc_sent, vc_dropped;
} sim_t;
extern sim_t SIM;

void sim_reset(void);
void sim_poll(void);					/* one 2 ms iteration of cyw43_utk_task's loop */
void sim_set_broker(int mode);
int  sim_inject(const char *topic, const char *payload, int frag);	/* broker -> robot; 0 if not routed */
int  sim_topic(const char *topic);
void sim_vc_status(int state, int speed, float hump, int fault);	/* what vc_task does: TMO_POL send */
int  sim_remote_cmd_take(void);					/* what vc_task does: poll mbf_remote_cmd */
int  sim_client_state(void);
extern void mqtt_utk_start(void);
#endif
