// THIS IS THE HEADER FOR YOUR WEEK 2 ASSIGNMENT. DO NOT CHANGE ANYTHING ALREADY WRITTEN, BUT YOU ARE FREE TO ADD YOUR OWN FUNCTIONS
// Make sure to implement all functions listed 
#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"
#include "IMU.h"
#include <mbed-os/thisThread.cpp>
#define PI 3.14159265

typedef struct{
    double yaw;
    double roll;
    double pitch;
} BNO055_ANGULAR_POSITION_typedef;

typedef struct {
    double x;
    double y;
    double z;
} BNO055_VECTOR_TypeDef;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} BNO055_QUATERNION_TypeDef;

class BNO055: public IMU
{
public:

    public:
    BNO055(I2C &i2c, uint8_t addr) noexcept; // Constructor taking in i2c object and address

    BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept; // Overloaded to take a specific reset pin 
                                                              // Implement both of these in the cpp

    void init() noexcept; // Initialize the BNO055    

    void reset() noexcept; // Quickly reset the BNO055

    /** Get Accel data
     * @param double type of 3D data address
     */
    void get_accel(BNO055_VECTOR_TypeDef *la);

    /** Get Gyro data
     * @param double type of 3D data address
     */
    void get_gyro(BNO055_VECTOR_TypeDef *gr);

    /** Change fusion mode
      * @param fusion mode
      * @return none
      */
    void change_fusion_mode(uint8_t mode);

    IMU::EulerAngles read() override;

    IMU::EulerAngles getImuAngles() override;

    /** Get Quaternion XYZ&W
     * @param int16_t type of 4D data address
     */
    void get_quaternion(BNO055_QUATERNION_TypeDef *qua);

    /** Get Angular position from quaternion
     *  @param double type of 3D data address
     */
    void get_angular_position_quat(IMU::EulerAngles *an_pos);

protected:

    I2C *_i2c_p;
    I2C &_i2c;
    DigitalOut _res;   // working buffer
    uint8_t  chip_addr;
    int cantReadDataCount;    
    
private:

    char     dt[10];      // working buffer
    uint8_t _addr_write;
    uint8_t _addr_read;

    IMU::EulerAngles imuAngles;

};

// Quaternion data registers
#define BNO055_QUATERNION_W_LSB 0x20

#endif