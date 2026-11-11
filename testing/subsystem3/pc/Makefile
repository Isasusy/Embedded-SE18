SRC = ../../../app_program/subsystem3
run: t
	./t
t: test_s3.c $(SRC)/line_algo.c $(SRC)/barcode_algo.c
	$(CC) -std=gnu11 -O1 -Wall -Wextra -o $@ test_s3.c $(SRC)/line_algo.c $(SRC)/barcode_algo.c -lm
clean:
	rm -f t
