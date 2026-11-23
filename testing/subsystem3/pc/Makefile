SRC = ../../../app_program/subsystem3
run: t
	./t
t: test_s3.c $(SRC)/line_algo.c $(SRC)/barcode_algo.c
	$(CC) -std=gnu11 -O1 -Wall -Wextra -o $@ test_s3.c $(SRC)/line_algo.c $(SRC)/barcode_algo.c -lm
clean:
	rm -f t

# barcode character -> direction table (set from the dashboard over MQTT)
map: test_bd_map
	./test_bd_map
test_bd_map: test_bd_map.c $(SRC)/barcode_algo.c $(SRC)/barcode_algo.h
	$(CC) -std=gnu11 -O1 -Wall -Wextra -o $@ test_bd_map.c $(SRC)/barcode_algo.c -lm
