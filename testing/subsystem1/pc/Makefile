SRC = ../../../app_program/subsystem1
run: t
	./t
t: test_telemetry.c $(SRC)/telemetry.c
	$(CC) -std=gnu11 -O1 -Wall -Wextra -o $@ test_telemetry.c $(SRC)/telemetry.c
clean:
	rm -f t
