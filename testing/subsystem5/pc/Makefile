SRC = ../../../app_program/subsystem5
run: t
	./t
t: test_s5.c $(SRC)/vc_logic.c $(SRC)/scan_algo.c
	$(CC) -std=gnu11 -O1 -Wall -Wextra -o $@ test_s5.c $(SRC)/vc_logic.c $(SRC)/scan_algo.c -lm
clean:
	rm -f t
