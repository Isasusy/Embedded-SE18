# WiFi/MQTT harness for Subsystem 1: the real mqtt_utk.c + subsystem1.c + telemetry.c on a PC (see README.md).
ROOT = ../../..
APP  = $(ROOT)/app_program
B    = build
CFLAGS = -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter -Ifake -I$(ROOT)/lib/liblwip/sysdepend/pico_rp2040

all: run
# subsystem1.c uses an Arm "dmb" barrier: the copy for the PC drops the instruction but keeps the compiler barrier.
$(B)/app_program/subsystem1/subsystem1.c: $(APP)/subsystem1/subsystem1.c $(APP)/subsystem1/subsystem1.h $(APP)/subsystem1/telemetry.h $(APP)/app.h $(APP)/subsystem5/vc_msgs.h
	mkdir -p $(B)/app_program/subsystem1 $(B)/app_program/subsystem5
	cp $(APP)/app.h $(B)/app_program/ && cp $(APP)/subsystem1/subsystem1.h $(APP)/subsystem1/telemetry.h $(B)/app_program/subsystem1/
	cp $(APP)/subsystem5/vc_msgs.h $(B)/app_program/subsystem5/
	sed 's/__asm__ volatile ("dmb" ::: "memory")/__asm__ volatile ("" ::: "memory")/' $< > $@
wifi_sim: sim_main.c fake_net.c sim.h $(B)/app_program/subsystem1/subsystem1.c $(APP)/subsystem1/telemetry.c $(ROOT)/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
	$(CC) $(CFLAGS) -o $@ sim_main.c fake_net.c $(B)/app_program/subsystem1/subsystem1.c $(APP)/subsystem1/telemetry.c \
		$(ROOT)/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
run: wifi_sim
	-./wifi_sim connect; ./wifi_sim commands; ./wifi_sim barcode
	-for r in 20 100 200 400; do ./wifi_sim rtt $$r; done
	-./wifi_sim closed 30; ./wifi_sim closed 600; ./wifi_sim silent 180
clean:
	rm -rf $(B) wifi_sim
