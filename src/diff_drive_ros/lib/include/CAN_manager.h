#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#define receive_all true
#define receive_one false

#include "Arduino.h"
#include "ESP32CAN.h"
#include "CAN_config.h"
#include "hardwareconfig.h"

#define CAN_CHECK_LOST_FREQ 200

#define USING_MAIN_BOARD

#ifdef USING_MOTOR_CONTROLLER_BOARD
    enum CAN_TransmitDataType{
        //ID_send = 0,
        CommandReceived = 0,
        commandStatus = 1,
        velocity = 2,
        error_check = 7
    };

    enum CAN_ReceiveDataType{
        ID_received = 0,
        IncommingCommand = 1,
        velocity = 2,
        SickSignal = 3,
    };
#endif

#ifdef USING_MAIN_BOARD
    enum sendCommandStatus{
        pending = 0,
        have_sent = 1,
        sendAccepted = 3,
    };

    enum CAN_TransmitData{
        to_ID = ID_Convert,

        ID_require = 0,
        CommandRequire = 1,
        CommandReset = 2,
        SpeedRequire = 2,
        SickSignal = 3,
        SelectHead = 2,
        SelectSick = 3,
        cam_led =4
    };
    enum CAN_ReceiveData{
        FeedbackCommand = 0,
        CommandStatus = 1,
        velocity = 2,
        LineTrakingSensor = 3,
        oneBitSignal = 4,
        error_check = 7,

        RFIDstatus4 = 4,
        RFIDstatus3 = 3,
        RFIDstatus2 = 2,
        RFIDstatus1 = 1,
        RFIDstatus0 = 0,
        SICKstatus = 5,

        sensorsData = 2 
    };

    enum byteOrder
    {
        // -- RTC
        ID    = 0x00,
        BYTE0 = 0x00,
        BYTE1 = 0x01,
        BYTE2 = 0x02,
        BYTE3 = 0x03,
        BYTE4 = 0x04,
        BYTE5 = 0x05,
        BYTE6 = 0x06,
        BYTE7 = 0x07,
        // -- Main
        MAIN_ID                 = 0x00,
        MAIN_rec_analogVoltage0 = 0x00,
        MAIN_rec_analogVoltage1 = 0x01,
        MAIN_rec_analogCurrent0 = 0x02,
        MAIN_rec_analogCurrent1 = 0x03,
        MAIN_rec_stsButton_reset= 0x04,
        MAIN_rec_stsButton_power= 0x05,
        MAIN_rec_EMG_status     = 0x06,
        
        MAIN_send_sound_enb     = 0x01,
        MAIN_send_sound_type    = 0x02,
        MAIN_send_charge_write  = 0x03,
        MAIN_send_EMG_reset     = 0x04,
        MAIN_send_EMG_write     = 0x05,
        // -- OC
        OC_ID                   = 0x00,
        OC_rec_FeedbackCommand  = 0x00,
        OC_rec_commandStatus    = 0x01,
        OC_rec_sensorsData      = 0x02,
        OC_rec_error_check      = 0x07,
        OC_send_commandRequire  = 0x01,
        OC_send_commandReset    = 0x02,
        // -- HC
        HC_ID               = 0x00,
        HC_rec_ZONE_AHEAD   = 0x04,
        HC_rec_ZONE_BEHIND  = 0x05,
        HC_rec_conllision   = 0x06,
        HC_rec_status       = 0x07,
        HC_send_RGB1        = 0x01,
        HC_send_RGB2        = 0x02
    };

#endif

class CAN_manager
{
private:
    CAN_speed_t _baud_speed;
    gpio_num_t _tx;
    gpio_num_t _rx;
    CAN_frame_format_t _frame;
    uint32_t _can_id;
    uint8_t _send_size ;

public:

    CAN_manager(CAN_speed_t ,gpio_num_t ,gpio_num_t , CAN_frame_format_t ,uint32_t , uint8_t );
    void CAN_prepare(void);
    // bool CAN_ReceiveFrom(int);
    uint32_t CAN_ReceiveFrom(void); 
    uint8_t GetByteReceived(unsigned char);


    bool CAN_Send();
    void SetByteTransmit(uint8_t,unsigned char);
    void SetIDTransmit(uint32_t);

    ~CAN_manager();
};

#endif