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
| robot/se18/cmd | laptop -> robot | start / stop / reset |

Build: `make WIFI=cyw43 NET=lwip MQTT=1 -j8` with `config/wifi_secrets.h` filled in (SSID, password, broker IP = the
laptop's IP on the same 2.4 GHz network).
