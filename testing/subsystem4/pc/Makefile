# PC-side unit test for the IMU algorithm (no Pico, no kernel needed).
#   make run
CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -O1 -g -I../../../app_program/subsystem4
SRC      = ../../../app_program/subsystem4

test_imu_algo: test_imu_algo.c $(SRC)/imu_algo.c $(SRC)/imu_algo.h $(SRC)/imu_config.h $(SRC)/imu_msgs.h
	$(CC) $(CFLAGS) -o $@ test_imu_algo.c $(SRC)/imu_algo.c -lm

run: test_imu_algo
	./test_imu_algo

clean:
	rm -f test_imu_algo
