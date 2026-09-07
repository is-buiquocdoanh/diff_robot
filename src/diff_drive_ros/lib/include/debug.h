#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#ifdef DEBUG_BY_UDP

#define NETWORK_NAME "STI_Vietnam_No8"
#define NETWORK_PASSWORD "66668888"
#define UDP_ADDRESS "192.168.1.31"
#define UDP_RECEIVE_PORT    8888
#define UDP_SEND_PORT       8000

extern WiFiUDP udpSever;

#endif
