#ifndef FAKE_LWIP_IP_ADDR_H
#define FAKE_LWIP_IP_ADDR_H
#include "lwip/apps/mqtt.h"
typedef struct { u32_t addr; u8_t type; } ip_addr_t;
#define IPADDR_TYPE_V4 0
#define ip_2_ip4(a) (a)
#define IP_SET_TYPE(a, t) ((a)->type = (t))
int ip4addr_aton(const char *cp, ip_addr_t *addr);
#endif
