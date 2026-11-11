/*
 *	subsystem3.h
 *	Subsystem 3 -- Line following and barcode decoding (Student ID 2503719 per the design review).
 *	Replaces the template's example Consumer (mbx_samples / app_stats / mtx_stats are gone).
 *	Other subsystems include it as "../subsystem3/subsystem3.h".
 */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../subsystem2/motion_msgs.h"		/* line_status_t */
#include "../subsystem5/vc_msgs.h"		/* nav_cmd_t, line_evt_t */

#define S3_PRI_LINE		16		/* priority band 16-17 (TEAM.md) */
#define S3_PRI_BARCODE		17

/* IF-11  S3 -> Motion Control : line_status_t, shared data + mutex (owned by S3). S2 reads with TMO_POL. */
IMPORT ID		mtx_line;
IMPORT line_status_t	app_line;

/* IF-16  S3 <-> Vehicle Controller : event flag flg_line (owned by S3) */
#define FLG_LINE_READY		0x1		/* calibration finished */
#define FLG_LINE_ALIVE		0x2		/* set every 5 ms; VC checks and clears it (NFR10) */
IMPORT ID		flg_line;

/* IF-12 / IF-13: S3 only SENDS to these message buffers (owned by the Vehicle Controller, S5).
   While they are 0 (S5 absent) events are printed instead. */
IMPORT ID		mbf_nav_cmd;
IMPORT ID		mbf_line_evt;

/* Private to subsystem 3 */
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	barcode_task(INT stacd, void *exinf);

#endif
