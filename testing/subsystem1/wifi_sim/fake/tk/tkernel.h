/* PC stand-in for micro T-Kernel: only the message-buffer calls Subsystem 1 uses. */
#ifndef FAKE_TKERNEL_H
#define FAKE_TKERNEL_H
#include <stdint.h>
#include <stdbool.h>
typedef int INT; typedef int ID; typedef int ER; typedef int TMO; typedef unsigned ATR; typedef int SZ;
typedef int PRI; typedef unsigned UW; typedef void (*FP)(void);
#define IMPORT extern
#define EXPORT
#define E_OK 0
#define E_TMOUT (-50)
#define TMO_POL 0
#define TA_TFIFO 0
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; } T_CMBF;
ID tk_cre_mbf(const T_CMBF *pk);
ER tk_snd_mbf(ID id, const void *msg, INT msgsz, TMO tmo);
INT tk_rcv_mbf(ID id, void *msg, TMO tmo);
#endif
