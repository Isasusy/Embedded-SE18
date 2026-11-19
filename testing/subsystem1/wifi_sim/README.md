# Subsystem 1 WiFi / MQTT harness (PC)

Runs the **real** `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `app_program/subsystem1/subsystem1.c` and
`telemetry.c` on a PC, on top of `fake_net.c`: a model of lwIP's MQTT client (lwIP 2.x defaults: 4 requests in
flight, 256-byte output ring, 5 s cyclic timer, 30 s request timeout, keep-alive watchdog 1.5 x 60 s, **no automatic
reconnect**, `ERR_ISCONN` while an attempt is in progress), the `MEMP_SYS_TIMEOUT` pool (4 slots above lwIP's own,
as in `lwipopts.h`; running out halts like `lwip_utk_assert`), TCP with a chosen round-trip time, a broker, and
micro T-Kernel message buffers. It does **not** test the radio, DHCP, or real lwIP code: that needs the Pico.

    make            # build, run every scenario on the current code
    VERBOSE=1 ./wifi_sim closed 30     # show the robot's console lines
    RTT=300 ./wifi_sim silent 120      # any scenario with another round-trip time

| Scenario | What it does |
|---|---|
| `connect` | 60 s normal running: subscriptions, 2 Hz status, 1 Hz heartbeat, JSON format, latency |
| `commands` | start/stop/reset on `robot/se18/cmd` (plain, JSON, fragmented, junk, long, flood) -> `mbf_remote_cmd` |
| `barcode` | barcode reads -> `robot/se18/barcode` |
| `rtt N` | 60 s with N ms round trip and a state-change burst + barcode every 5 s |
| `closed S` | broker process stopped for S s (port refuses), then back |
| `silent S` | broker host unreachable for S s (laptop leaves the hotspot), then back |

`subsystem1.c` is compiled from a copy without the Arm `dmb` instruction (the compiler barrier is kept).

## Fixes found with this harness (applied to the repo)
* `mqtt_utk.c` reconnect: after a connection that was up is lost, the retry timer and the poll loop both called
  `mqtt_utk_start()`, flooding the console with `mqtt: mqtt_client_connect() failed, err=-10` and filling the 4-slot
  timer pool. `connecting` now stays set until the retry runs.
* `subsystem1.c` status retry: a status lwIP refused (all 4 requests waiting for a TCP ACK) was dropped. It is now
  kept and sent on the next 100 ms call.
