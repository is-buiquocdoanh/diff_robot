#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "hardwareconfig.h"
#include "controll_config.h"
#include "CAN_manager.h"
#include <WiFi.h>  

struct Controll_OC{
    uint8_t CAN_address;
    uint8_t command = DO_NOTHING;
    uint8_t status = have_sent;
    bool resetError = false;

    Controll_OC(int adr) {CAN_address = adr;}
    ~Controll_OC();
};


struct CAN_SEND{ // - HC
    uint16_t id;
    uint8_t byte0;
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
    uint8_t byte3 = 0;
    uint8_t byte4 = 0;
    uint8_t byte5 = 0;
    uint8_t byte6 = 0;
    uint8_t byte7 = 0;

    CAN_SEND() {}
    ~CAN_SEND(){}
};

struct CAN_RECEIVED{ // - HC
    uint8_t idSend;
    uint8_t byte0 = 0;
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
    uint8_t byte3 = 0;
    uint8_t byte4 = 0;
    uint8_t byte5 = 0;
    uint8_t byte6 = 0;
    uint8_t byte7 = 0;

    CAN_RECEIVED() {}
    ~CAN_RECEIVED(){}
};

class Main_controller
{
    private:
        
        CAN_manager* CAN_main = NULL;

        unsigned long saveTime_checkCAN = 0;
        unsigned long saveTime_checkReset = 0;

        // - for reset EMG
        int flag_reseting = 0;
        unsigned long saveTime_checkReseted = 0;
        unsigned long saveTime_sendCAN = 0;

        unsigned long saveTime_light = 0;
        bool sts_light = false;
    public:
        Main_controller(CAN_manager*);
        ~Main_controller();

        CAN_SEND* send_CAN = new CAN_SEND();
        CAN_RECEIVED* received_CAN = new CAN_RECEIVED();

        bool is_receivedCAN = false;
        bool is_sendCAN = false;

        unsigned long saveTime_LED_ROS_SEND = 0;
        unsigned long saveTime_LED_ROS_RECEIVED = 0;
        unsigned long saveTime_LED_CAN_SEND = 0;
        unsigned long saveTime_LED_CAN_RECEIVED = 0;

        bool sts_LED_ROS_RECEIVED = false;
        bool sts_LED_ROS_SEND = false;
        bool sts_LED_CAN_SEND = false;
        bool sts_LED_CAN_RECEIVED = false;

        bool stsSend_CAN = 0;
        bool stsRev_CAN = 0;

        void led_start();
        void led_loop();
        void led_blink();
        void led_control(bool, bool, bool, bool);

        void CANReceiveHandle();
        bool CANTransmitHandle();

        void init_main();
        void Run_Send_CAN();

        bool getBit_fromInt(int, int);
        int bytes_to_int(uint8_t, uint8_t);
        uint8_t int_to_byte0(int);
        uint8_t int_to_byte1(int);
};
#endif