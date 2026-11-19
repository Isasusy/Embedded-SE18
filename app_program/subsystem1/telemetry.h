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
#define S1_TOPIC_OBSTACLE	"robot/se18/obstacle"	/* every obstacle profiling sweep */

/* each returns the text length (no terminating NUL counted, but one is written), or -1 if `cap` is too small */
int tm_format_status(char *buf, int cap, const robot_status_t *s);
int tm_format_heartbeat(char *buf, int cap, uint32_t seq, uint32_t t_ms);
/* {"code":"A","cmd":"LEFT","n":3,"pattern":"...","t":...}; code "" = not decoded */
int tm_format_barcode(char *buf, int cap, const char *code, int nav, const char *pattern, uint32_t n, uint32_t t_ms);

/* {"n":1,"deg":[30,60,90,120,150],"mm":[412,230,198,245,-1],"found":true,"dist_mm":198,"width_mm":120,
    "left_clear_mm":300,"right_clear_mm":90,"side":"LEFT","t":...}  (mm -1 = no echo; side LEFT / RIGHT / NONE) */
int tm_format_obstacle(char *buf, int cap, const obstacle_profile_t *p, uint32_t n);

/* accepts plain text ("start") or JSON ({"cmd":"STOP"}), any case. Returns remote_cmd_type_t or -1 */
int tm_parse_cmd(const char *payload, int len);

const char *tm_state_name(int state);
const char *tm_fault_name(int fault);

#endif
