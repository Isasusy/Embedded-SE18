/*
 *	subsystem3.c
 *	Subsystem 3 -- creates its objects and starts its two tasks.
 *	APIs shown: tk_cre_flg, tk_cre_mtx
 */
#include "subsystem3.h"
#include "drv_line.h"

EXPORT ID		flg_line;	/* IF-16 */
EXPORT ID		mtx_line;	/* IF-11 */
EXPORT line_status_t	app_line;

/* Owned by the Vehicle Controller (S5). Weak defaults (0) so S3 links and runs alone. */
__attribute__((weak)) ID	mbf_nav_cmd;
__attribute__((weak)) ID	mbf_line_evt;

EXPORT ER subsystem3_init(void)
{
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_line = tk_cre_flg(&cflg);
	if (flg_line < E_OK) {
		return flg_line;
	}

	cmtx.mtxatr = TA_INHERIT;		/* Motion Control (higher priority) reads app_line */
	mtx_line = tk_cre_mtx(&cmtx);
	if (mtx_line < E_OK) {
		return mtx_line;
	}
	line_adc_init();			/* once, before either task runs: both share the ADC */
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task(line_task, S3_PRI_LINE, 0, "line_task");
	if (tid < E_OK) {
		return tid;
	}
	tid = app_start_task(barcode_task, S3_PRI_BARCODE, TP_PRC1, "barcode_task");
	return (tid < E_OK) ? tid : E_OK;
}
