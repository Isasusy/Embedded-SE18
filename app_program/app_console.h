/*
 *	app_console.h -- one switch for ALL console output of the five subsystems.
 *	APP_CONSOLE 1: printing to the USB serial console (what you use while the cable is plugged in).
 *	APP_CONSOLE 1: no printing at all. Use it for runs on battery with the USB cable unplugged: if the console
 *	               blocks when no PC is listening, the printing tasks would freeze and the car would stop.
 */
#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H
#define APP_CONSOLE	1
#endif
