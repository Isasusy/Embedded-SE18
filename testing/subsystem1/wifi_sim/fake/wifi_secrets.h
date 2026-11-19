/* Test values only (the real config/wifi_secrets.h is never read by this harness). */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H
#define WIFI_SSID        "test-ssid"
#define WIFI_PASSWORD    "test-pass"
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"
#endif
