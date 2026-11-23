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

# closed-loop line-following simulation: new steering (asserts), and the original code for comparison
OLD_S3 ?= old_s3
sim: test_line_sim
	./test_line_sim
test_line_sim: test_line_sim.c $(SRC)/line_algo.c $(SRC)/line_algo.h $(SRC)/line_config.h
	$(CC) -std=gnu11 -O1 -Wall -Wextra -I$(SRC) -o $@ test_line_sim.c $(SRC)/line_algo.c -lm
sim_old: test_line_sim.c
	$(CC) -std=gnu11 -O1 -Wall -Wextra -DOLD -I$(OLD_S3) -o test_line_sim_old test_line_sim.c $(OLD_S3)/line_algo.c -lm
	./test_line_sim_old
