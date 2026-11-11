/*
 *	telemetry.h / telemetry.c -- text formats for the MQTT link: status JSON, heartbeat JSON, remote command parser.
 *	Plain C, no network and no kernel calls: testable on a PC. No printf / snprintf (the kernel's console printf has no
 *	floating point, and these run inside the WiFi service task, so they are kept tiny and allocation-free).
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "../subsystem5/vc_msgs.h"

#define S1_TOPIC_STATUS		"robot/se18/status"	/* 2 Hz, NFR5 */
#define S1_TOPIC_HEARTBEAT	"robot/se18/heartbeat"	/* 1 Hz, NFR5 */
#define S1_TOPIC_CMD		"robot/se18/cmd"	/* subscribed: START / STOP / RESET */
#define S1_TOPIC_BARCODE	"robot/se18/barcode"	/* every barcode read, valid or not */

/* each returns the text length (no terminating NUL counted, but one is written), or -1 if `cap` is too small */
int tm_format_status(char *buf, int cap, const robot_status_t *s);
int tm_format_heartbeat(char *buf, int cap, uint32_t seq, uint32_t t_ms);
/* {"code":"A","cmd":"LEFT","n":3,"pattern":"...","t":...}; code "" = not decoded */
int tm_format_barcode(char *buf, int cap, const char *code, int nav, const char *pattern, uint32_t n, uint32_t t_ms);

/* accepts plain text ("start") or JSON ({"cmd":"STOP"}), any case. Returns remote_cmd_type_t or -1 */
int tm_parse_cmd(const char *payload, int len);

const char *tm_state_name(int state);
const char *tm_fault_name(int fault);

#endif
