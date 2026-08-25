#pragma once

#include "PinNames.h"
#include "mbed.h"
#include "util/communications/DJIRemote2.h"
#include "util/communications/referee/ref_serial.h"
#include "util/motor/DJIMotor.h"
#include <us_ticker_api.h>

class BaseRobot {
  public:
    struct Config {
        PinName remote_tx_pin = NC; 
		PinName remote_rx_pin = PA_10;

        PinName referee_tx_pin = PC_10;
        PinName referee_rx_pin = PC_11;

        PinName can1_rx_pin = PA_11;
        PinName can1_tx_pin = PA_12;

        PinName can2_rx_pin = PB_12;
        PinName can2_tx_pin = PB_13;

        PinName led0_pin = PB_0;
        PinName led1_pin = PC_1;
        PinName led2_pin = PC_0;
    };

    DJIRemote2 remote_;
    Referee referee_;

    CANHandler canHandler1_;
    CANHandler canHandler2_;
    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;

    // Remote control variables
    float scalar = 1;
    float jx = 0; // -1 to 1
    float jy = 0; // -1 to 1
    // Pitch, Yaw
    float jpitch = 0; // -1 to 1
    float jyaw = 0; // -1 to 1
    float myaw = 0;
    float mpitch = 0;
    int pitchVelo = 0;
    // joystick tolerance
    float tolerance = 0.05;
    // Keyboard Driving
    float mult = 0.7;
    float omega_speed = 0;
    float max_linear_vel = 0;

    // drive and shooting mode
    // "o" - joystick
    // "u" - drive
    // "d" - beyblade
    // "m" - off
    // "y" - yaw-align

    // "o" - joystick
    // "d" - flywheel
    // "m" - flywheel off
    char drive = 'o'; //default o when using joystick 
    char shot = 'o'; //default o when using joystick
    bool cv_enabled_ = false;

    // clang-format off
    BaseRobot(const Config &config)
        : remote_(config.remote_tx_pin, config.remote_rx_pin),
          referee_(config.referee_tx_pin, config.referee_rx_pin), 
          canHandler1_(config.can1_rx_pin, config.can1_tx_pin),
          canHandler2_(config.can2_rx_pin, config.can2_tx_pin),
          led0_(config.led0_pin),
          led1_(config.led1_pin),
          led2_(config.led2_pin)
    {};
    // clang-format on

    virtual ~BaseRobot() = default;

    virtual void init() = 0;
    virtual void periodic(const unsigned long dt_us) = 0;
    virtual void end_of_loop() {};

    // default 1000hz main loop. Can be overriden
    virtual unsigned int main_loop_dt_ms() { return 1; };

    virtual void main_loop() {
        unsigned long loop_clock_us = us_ticker_read();
        unsigned long prev_loop_time_us = loop_clock_us;
        unsigned long prev_remote_time_us = loop_clock_us;

        unsigned long main_loop_dt_ms = this->main_loop_dt_ms();

        // Init all constants, subsystems, sensors, IO, etc.
        // Each can message has an id and data, and djimotor ids start from 0x201(m3508 id 1) and
        // end at 0x20D (gm6020 id 8), there is an overlap of 4 motors (M3508 id 5-8 and gm6020 id
        // 1-4), so that is why we have 12 values only
        canHandler1_.registerCallback(0x201, 0x20D, DJIMotor::getCan1Feedback);
        canHandler2_.registerCallback(0x201, 0x20D, DJIMotor::getCan2Feedback);
        DJIMotor::setCanHandlers(&canHandler1_, &canHandler2_);

        init();

        while (true) {
            loop_clock_us = us_ticker_read();

            // 20 ms remote read
            if ((loop_clock_us - prev_remote_time_us) / 1000 >= 15) {
                remote_.update();
                remoteRead();
                prev_remote_time_us = loop_clock_us;
            }

            if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms) {
                // Add subsystems in periodic
                led0_ = !led0_;

                periodic(loop_clock_us - prev_loop_time_us);
                prev_loop_time_us = loop_clock_us;

                // Motor updates
                DJIMotor::sendValues();
            }
            // Add sensors updates in your end of loop
            end_of_loop();

            canHandler1_.readAllCan();
            canHandler2_.readAllCan();
        }
    }

    void remoteRead()
    {

        //Driving input 
        //TODO: Read joystick x and y axis values and assign them to respective j-variable. REMINDER: jx jy is for left joystick, the others for right joystick
        //FYI They're all set to zero for now so that it compiles
        jx = remote_.getJoystickValue(DJIRemote2::Joystick::LEFT_HORIZONTAL);
        jy = remote_.getJoystickValue(DJIRemote2::Joystick::LEFT_VERTICAL);
        //Pitch, Yaw
        jpitch = remote_.getJoystickValue(DJIRemote2::Joystick::RIGHT_HORIZONTAL);
        jyaw = remote_.getJoystickValue(DJIRemote2::Joystick::RIGHT_VERTICAL);


        //Deadzones provided for you 
        jx = (abs(jx) < tolerance) ? 0 : jx;
        jy = (abs(jy) < tolerance) ? 0 : jy;
        jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
        jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
        
        
        //Bounding the four j variables
        //TODO: Make sure they're all on [-1, 1] range
        jx = fmax(fmin(1, jx), -1);
        jy = fmax(fmin(1, jy), -1);
        jpitch = fmax(fmin(1, jpitch), -1);
        jyaw = fmax(fmin(1, jyaw), -1);
    }
};