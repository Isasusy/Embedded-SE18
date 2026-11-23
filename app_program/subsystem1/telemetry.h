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
#define S1_TOPIC_HUMP		"robot/se18/hump"	/* every hump the IMU measures, once */
#define S1_TOPIC_BARCODE_MAP	"robot/se18/barcode_map"	/* the barcode letter -> direction table in use */
#define S1_TOPIC_CONFIG		"robot/se18/config"		/* calibration in use: {"track_mm":138.5} */

/* each returns the text length (no terminating NUL counted, but one is written), or -1 if `cap` is too small */
int tm_format_status(char *buf, int cap, const robot_status_t *s);
int tm_format_heartbeat(char *buf, int cap, uint32_t seq, uint32_t t_ms);
/* {"code":"A","cmd":"LEFT","n":3,"pattern":"...","t":...}; code "" = not decoded */
int tm_format_barcode(char *buf, int cap, const char *code, int nav, const char *pattern, uint32_t n, uint32_t t_ms);

/* {"n":1,"deg":[30,60,90,120,150],"mm":[412,230,198,245,-1],"found":true,"dist_mm":198,"width_mm":120,
    "left_clear_mm":300,"right_clear_mm":90,"side":"LEFT","t":...}  (mm -1 = no echo; side LEFT / RIGHT / NONE) */
int tm_format_obstacle(char *buf, int cap, const obstacle_profile_t *p, uint32_t n);

/* {"n":2,"peak_mm":18.4,"max_hump_mm":22.3,"t":...}  n = humps since boot */
int tm_format_hump(char *buf, int cap, uint32_t n, float peak_mm, float max_hump_mm, uint32_t t_ms);

/* accepts plain text ("start") or JSON ({"cmd":"STOP"}), any case. Returns remote_cmd_type_t or -1 */
int tm_parse_cmd(const char *payload, int len);

/* ---- barcode direction table (S3's bd_map_*, changeable from the dashboard) ----
 * Command on S1_TOPIC_CMD: "map A=LEFT B=RIGHT C=STRAIGHT D=UTURN E=STOP F=NONE" (any case, spaces or commas).
 * Characters A-Z / 0-9; directions LEFT RIGHT STRAIGHT UTURN (or U-TURN) STOP NONE.
 * Returns the number of pairs written to chars[] / navs[] (nav -1 = NONE), or -1 if it is not a valid map command. */
/* "spin" (also via tm_parse_cmd -> REMOTE_SPIN): turn calibration, 360 deg on the spot, only in IDLE.
 * "cal track=138.5": set Motion Control's effective track (mc_set_track). Returns 1 + *track_mm, or -1. */
int tm_parse_cal(const char *payload, int len, float *track_mm);
/* {"track_mm":138.5} */
int tm_format_config(char *buf, int cap, float track_mm);

#define TM_MAP_N	36			/* same as BD_MAP_N: A-Z then 0-9 */
int tm_parse_map(const char *payload, int len, char *chars, int *navs, int max);
/* {"A":"LEFT","B":"RIGHT",...}: only characters that have a direction. nav_by_index: -1 = none */
int tm_format_map(char *buf, int cap, const int8_t nav_by_index[TM_MAP_N]);

const char *tm_state_name(int state);
const char *tm_fault_name(int fault);

#endif
