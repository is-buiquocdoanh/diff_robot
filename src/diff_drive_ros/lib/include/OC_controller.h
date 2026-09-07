#ifndef OC_CONTROLLER_H
#define OC_CONTROLLER_H

#include "CAN_manager.h"
#include "hardwareconfig.h"
#include "controll_config.h"
class OC_controller
{
private:
    unsigned long time_gui;
    /**
    * @brief          : luu gia tri nhan/gui cua mang giao tiep CAN
    * @details        : can_id: id can cua mach
                        cmd,cmd_status: function va trang thai (0:lo lung || 1:o tren || 2:o duoi)
                        status_r: hoan thanh lenh cmd
                        err_lx: bao loi ban nang
    * @note           : 
    */
    unsigned char   cmd = commands::DO_NOTHING,
                    cmd_status = processStatus::COMMAND_DONE,
                    err_lx = ISOK;        
    CAN_manager* CANctr = NULL;
public:
    OC_controller(CAN_manager*);
    ~OC_controller();
    void setCommand(unsigned char _cmd) {cmd = _cmd;}
    void setCommandStatus(unsigned char _cmd_status) {cmd_status = _cmd_status;}
    void setError(unsigned char _err_lx) {err_lx = _err_lx;}

    unsigned char getError() {return err_lx;}
    unsigned char getCommandStatus() {return cmd_status;}
    unsigned char getCommand() {return cmd;}
    bool getSensor(int _pin) {return digitalRead(_pin);}
    
    bool the_three_timer(unsigned int timer_num, unsigned int set_value){
        static unsigned int timer_value[3]={0,0,0};
        int t = millis();
        if (t- timer_value[timer_num] >= set_value){
            timer_value[timer_num] = t;
            return true;
        }
        else{
            return false;
        }
    }

    void LiftUp();
    void LiftDown();
    void ConveyorIn();
    void ConveyorOut();

    void stop();
    void setupConv();
    void OC_CAN_Transmit();
    void OC_CAN_Receive();
    void OC_loop();
    void debuge();
};


#endif