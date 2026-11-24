# Subsystem 1: Wi-Fi + MQTT telemetry (set-up, 11 Nov)

The telemetry code (app_program/subsystem1) runs inside the template's Wi-Fi task through two hooks added to
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` (lines marked SE18; the original is kept as mqtt_utk.c.orig).
This changes shared library code: tell the team / instructor.

Topics (laptop broker, monitor with MQTT Explorer):

| Topic | Direction | Content |
|---|---|---|
| robot/se18/barcode | robot -> laptop | every barcode read: {"code":"A","cmd":"LEFT","n":3,"pattern":"...","t":...} |
| robot/se18/status | robot -> laptop | state, speed, max hump, fault, every 0.5 s and on every state change |
| robot/se18/heartbeat | robot -> laptop | every 1 s |
| robot/se18/obstacle | robot -> laptop | every obstacle profile (servo 30..150 deg in 5 deg steps, 25 rays): {"n":1,"deg":[30,35,...,150],"mm":[...],"found":true,"dist_mm":...,"width_mm":...,"left_clear_mm":...,"right_clear_mm":...,"side":"LEFT","t":...} |
| robot/se18/cmd | laptop -> robot | start / stop / reset |

Build: `make WIFI=cyw43 NET=lwip MQTT=1 -j8` with `config/wifi_secrets.h` filled in (SSID, password, broker IP = the
laptop's IP on the same 2.4 GHz network).

## Added 23 Nov
| Topic | Direction | Content |
|---|---|---|
| robot/se18/hump | robot -> laptop | every hump the IMU measures, once: {"n":2,"peak_mm":18.4,"max_hump_mm":22.3,"t":...} |
| robot/se18/barcode_map | robot -> laptop | barcode letter -> direction in use, on change and every 5 s: {"A":"LEFT","B":"RIGHT",...} |
| robot/se18/cmd | laptop -> robot | also `map A=LEFT B=RIGHT C=STRAIGHT D=UTURN E=STOP F=NONE` (letters A-Z / 0-9) |

`robot/se18/status` also has `"waiting":[...]` (MOTORS, LINE, IMU, WIFI) while the state is INIT.
The barcode directions are not saved: after a power-up the robot is back to A LEFT, B RIGHT, C STRAIGHT, D UTURN
(line_config.h); press Send to robot on the dashboard again.
