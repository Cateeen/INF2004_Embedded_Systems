#ifndef INF2004_LWIPOPTS_H
#define INF2004_LWIPOPTS_H

#define NO_SYS                      1
#define LWIP_DHCP                   1
#define LWIP_DNS                    1
#define LWIP_RAW                    1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

/* lwIP MQTT uses an additional cyclic timer. */
#define MEMP_NUM_SYS_TIMEOUT        16

#endif
