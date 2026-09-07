/**
 * File : hardwareconfig.h
 * Version : 8.2.0
 * Date    : 17/06/2020
 * Author  : AnDX
 * Description :
 * 
 *******************************************************/

#ifndef __HARDWARECONFIG_H
#define __HARDWARECONFIG_H
#include <Arduino.h>
#include "CAN_config.h"

#define CONTROL_PROCESS
#define CAN_DEBUG
#define SERVER_PROCESS
#define SERVER_DEBUG

#define ID_Convert  0x01
#define ID_HC       0x02
#define ID_MAIN     0x03
#define ID_OC       0x04

#define timerCAN  0
#define timerCTR  20
#define timerPOW  1000

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS // CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_Convert
#define CAN_SEND_SIZE   8

#define ID_RFID_HEAD    1
#define ID_RFID_BEHIND  2

//------------------------------ Pin define -----------------------------
#define PIN_LED_CAN_RECEIVED 25
#define PIN_LED_CAN_SEND 26

#define PIN_LED_ROS_RECEIVED 27
#define PIN_LED_ROS_SEND 33

#define PIN_INPUT1    35  
#define PIN_INPUT2    34  
//--------------------------------------------------------------------------------------------------------------------
// //Sever connection config
#define NETWORK_NAME "STI_Vietnam_No8"
#define NETWORK_PASSWORD "66668888"
#define UDP_ADDRESS "192.168.1.31" // PC 
#define UDP_RECEIVE_PORT    8888
#define UDP_SEND_PORT       8000
#define MAXIMUM_RECEIVED_PACKET 500
#define SIZE_TRANSMIT_PACKET 30
#define SIZE_ROS_PACKET 5
#define SIZE_REPLY_PACKET 6

// #define DEBUG_BY_UDP
#define DEBUG_BY_SERIAL
#if defined(DEBUG_BY_SERIAL)
#define debugSerial Serial
#define LOG_BEGIN(BAUD) debugSerial.begin(BAUD);\
                        debugSerial.println("Log start!!  ε=ε=(づ￣ 3￣)づ  ε=ε=ε=┏(゜ロ゜;)┛ ")
#define LOG_MESS(...) debugSerial.println(__VA_ARGS__)
#define LOG_MESS_STRING(...) debugSerial.println((String) __VA_ARGS__)
#define LOG_NUM(NUM) debugSerial.println((String) #NUM + " = " + NUM)
#elif defined(DEBUG_BY_UDP)

#define LOG_BEGIN(BAUD) WiFi.onEvent(WiFiEvent);\
                        WiFi.begin(NETWORK_NAME, NETWORK_PASSWORD);\
                        logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                        logSever->println("Logging udp - begin  ε=ε=(づ￣ 3￣)づ  ε=ε=ε=┏(゜ロ゜;)┛ ");\
                        logSever->endPacket()
#define LOG_MESS(...)   logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                        logSever->println(__VA_ARGS__);\
                        logSever->endPacket()
#define LOG_MESS_STRING(...)    logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                                logSever->println((String) __VA_ARGS__);\
                                logSever->endPacket()
#define LOG_NUM(NUM)    logSever->beginPacket(UDP_ADDRESS, UDP_SEND_PORT);\
                        logSever->println((String) #NUM + " = " + NUM);\
                        logSever->endPacket()
#else
#define LOG_BEGIN(BAUD)
#define LOG_MESS(...)
#define LOG_MESS_STRING(...)
#define LOG_NUM(NUM)
#endif

#endif
