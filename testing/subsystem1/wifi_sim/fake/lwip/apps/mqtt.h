/* PC model of lwIP's MQTT client API (lwip/apps/mqtt.h), only what mqtt_utk.c uses. */
#ifndef FAKE_LWIP_MQTT_H
#define FAKE_LWIP_MQTT_H
#include <stdint.h>
#include <stddef.h>
typedef uint8_t u8_t; typedef uint16_t u16_t; typedef uint32_t u32_t; typedef int8_t err_t;
#define ERR_OK        0
#define ERR_MEM      -1
#define ERR_TIMEOUT  -3
#define ERR_VAL      -6
#define ERR_ISCONN  -10
#define ERR_CONN    -11
typedef enum {
	MQTT_CONNECT_ACCEPTED = 0, MQTT_CONNECT_REFUSED_PROTOCOL_VERSION = 1, MQTT_CONNECT_REFUSED_IDENTIFIER = 2,
	MQTT_CONNECT_REFUSED_SERVER = 3, MQTT_CONNECT_REFUSED_USERNAME_PASS = 4, MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_ = 5,
	MQTT_CONNECT_DISCONNECTED = 256, MQTT_CONNECT_TIMEOUT = 257
} mqtt_connection_status_t;
typedef struct mqtt_client_s mqtt_client_t;
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
typedef void (*mqtt_incoming_publish_cb_t)(void *arg, const char *topic, u32_t tot_len);
typedef void (*mqtt_incoming_data_cb_t)(void *arg, const u8_t *data, u16_t len, u8_t flags);
typedef void (*mqtt_request_cb_t)(void *arg, err_t err);
struct mqtt_connect_client_info_t {
	const char *client_id, *client_user, *client_pass; u16_t keep_alive;
	const char *will_topic, *will_msg; u8_t will_msg_len, will_qos, will_retain;
};
#define MQTT_DATA_FLAG_LAST 1
mqtt_client_t *mqtt_client_new(void);
u8_t  mqtt_client_is_connected(mqtt_client_t *client);
err_t mqtt_client_connect(mqtt_client_t *client, const void *ipaddr, u16_t port, mqtt_connection_cb_t cb, void *arg,
			  const struct mqtt_connect_client_info_t *client_info);
err_t mqtt_publish(mqtt_client_t *client, const char *topic, const void *payload, u16_t payload_length, u8_t qos,
		   u8_t retain, mqtt_request_cb_t cb, void *arg);
err_t mqtt_sub_unsub(mqtt_client_t *client, const char *topic, u8_t qos, mqtt_request_cb_t cb, void *arg, u8_t sub);
#define mqtt_subscribe(c, t, q, cb, a) mqtt_sub_unsub(c, t, q, cb, a, 1)
void  mqtt_set_inpub_callback(mqtt_client_t *client, mqtt_incoming_publish_cb_t pub_cb,
			      mqtt_incoming_data_cb_t data_cb, void *arg);
#endif
