CC ?= gcc
SRC = ../../../app_program/subsystem2

run: test_motion_ctrl
	./test_motion_ctrl

test_motion_ctrl: test_motion_ctrl.c $(SRC)/motion_ctrl.c
	$(CC) -std=c11 -O1 -Wall -Wextra -o $@ test_motion_ctrl.c $(SRC)/motion_ctrl.c -lm

clean:
	rm -f test_motion_ctrl
