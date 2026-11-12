/*
 *	app_console.h -- one switch for ALL console output of the five subsystems.
 *	APP_CONSOLE 1: printing to the USB serial console (what you use while the cable is plugged in).
 *	APP_CONSOLE 0: no printing at all. Saves the time spent formatting text on runs on battery. (The USB console
 *	               does not block when no PC is listening: lib/libtm/.../usb_console.c drops the output instead.)
 */
#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H
#define APP_CONSOLE	1
#endif
