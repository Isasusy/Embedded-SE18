#ifndef FAKE_TMONITOR_H
#define FAKE_TMONITOR_H
typedef unsigned char UB;
int tm_printf(const UB *fmt, ...);
int tm_sprintf(UB *buf, const UB *fmt, ...);
#endif
