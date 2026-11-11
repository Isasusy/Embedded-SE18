/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi and MQTT telemetry (Student ID 2503802 per the design review).
 *	Replaces the template's example LED heartbeat (led_task, tid_led, GP19 LED are gone).
 *
 *	TEAM.md rule: NO student task may call cyw43 / lwIP / MQTT. So this subsystem has NO task. Its code runs INSIDE the
 *	kernel's WiFi service task (cyw43_utk_task), called from mqtt_utk.c through two hooks, s1_service() and s1_rx().
 *	See PATCH_mqtt_utk.md for the exact lines to add (this changes shared library code: agree it with the team
 *	and the instructor first, and it needs the NET=lwip MQTT=1 build).
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telemetry.h"

/* IF-14  Vehicle Controller -> S1 : robot_status_t, message buffer owned by S1. The VC sends with TMO_POL. */
#define S1_STATUS_MAXMSZ	((INT)sizeof(robot_status_t))
#define S1_STATUS_BUFSZ		(8 * (sizeof(robot_status_t) + 4))	/* depth 8 (Section 6.6) */
IMPORT ID	mbf_status;

/* IF-15  S1 -> Vehicle Controller : remote_cmd_t, message buffer owned by S5 (see ../subsystem5/subsystem5.h). */
IMPORT ID	mbf_remote_cmd;

/* The two hooks. Both are non-blocking and are called ONLY from the WiFi service task. */
typedef int (*s1_publish_fn)(void *ctx, const char *topic, const char *payload, int len);
IMPORT void	s1_service(s1_publish_fn publish, void *ctx, uint32_t now_ms);	/* call at least every 100 ms once MQTT is ONLINE */
IMPORT void	s1_rx(const char *topic, const char *payload, int len);		/* call for every received MQTT message */

#endif
