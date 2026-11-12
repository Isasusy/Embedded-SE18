# Patch for `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

**Status: applied.** The changed lines in `mqtt_utk.c` are marked `SE18` (hook declarations, `s1_pub`, the 100 ms
`s1_timeout` service chain, the `robot/se18/cmd` subscription and the `s1_rx` call); the original is kept as
`mqtt_utk.c.orig`. The steps below describe what was done. It changes shared library code, so the team and the
instructor should know (TEAM.md). Build with `NET=lwip MQTT=1`. Only the WiFi service task may run any of this.

1. Add near the top: `#include "../../../../app_program/subsystem1/subsystem1.h"` (adjust the relative path), and the
   subscription to `S1_TOPIC_CMD` where the client connects (next to the heartbeat setup).
2. **Receive**: in the function that handles an incoming message (the lwIP callback `mqtt_incoming_data_cb` or the
   equivalent), call
   ```c
   s1_rx(current_topic, (const char *)data, len);
   ```
   (if the payload can arrive in several fragments, collect it first; commands are under 128 bytes).
3. **Publish**: wrap the existing publish call in a function with this signature
   ```c
   static int s1_pub(void *ctx, const char *topic, const char *payload, int len)
   {
       return mqtt_publish((mqtt_client_t *)ctx, topic, payload, (u16_t)len, 0 /*qos*/, 0 /*retain*/, NULL, NULL) == ERR_OK ? 0 : -1;
   }
   ```
   and in the service loop, **only while the client is connected (ONLINE)** and at least every 100 ms:
   ```c
   s1_service(s1_pub, client, (uint32_t)<milliseconds since boot>);
   ```
4. Keep the existing heartbeat publish, or delete it: `s1_service` already publishes `robot/se18/heartbeat` every 1 s.
5. Do NOT call anything from `subsystem1.c` in any other task, and do not add a task for WiFi.

The VC sends `robot_status_t` to `mbf_status` every 500 ms and on every state change; `s1_service` formats and publishes it.
Reconnect / back-off (statechart 6.5) is whatever `mqtt_utk.c` already does; the robot keeps running while the link is down
because the VC never waits for telemetry (a full `mbf_status` just drops the oldest-not-yet-sent status).
