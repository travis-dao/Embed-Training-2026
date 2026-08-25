#include "ChassisSubsystem.h"
#include "util/motor/DJIMotor.h"
#include <cmath>
#include <stdexcept>

/**
 * @param radius radius in meters
 */
ChassisSubsystem::ChassisSubsystem(const Config &config)
    : power_limit(60),
      LF(config.left_front_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      RF(config.right_front_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      LB(config.left_back_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      RB(config.right_back_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      encoder(config.encoder),
      yawPhase{config.yaw_initial_offset_ticks}, // change Yaw to CCW +, and ranges from 0 to 360
      imu(config.imu),
      chassis_radius(config.radius),
      FF_Ks(config.speed_pid_ff_ks)
{
    LF.outputCap = 16000; // DJIMotor class has a max outputCap: 16384
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    setOmniKinematics(config.radius);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL; // m/s
    m_OmniKinematicsLimits.max_vOmega = 8; // rad/s

    PEAK_POWER_ALL = 10000;
    PEAK_POWER_SINGLE = 8000;


//    LF.setSpeedPID(2, 0, 0);
//    RF.setSpeedPID(2, 0, 0);
//    LB.setSpeedPID(2, 0, 0);
//    RB.setSpeedPID(2, 0, 0);
    // LF.setSpeedPID(3, 0, 0);
    // RF.setSpeedPID(3, 0, 0);
    // LB.setSpeedPID(3, 0, 0);
    // RB.setSpeedPID(3 , 0, 0);

    pid_LF.setPID(3.38, 0, 0);
    pid_RF.setPID(3.38, 0, 0);
    pid_LB.setPID(3.38, 0, 0);
    pid_RB.setPID(3.38, 0, 0);
    pid_align.setPID(5, 0.0, 0.5);
    pid_align.setOutputCap(1000);
    yaw_velo_gain = 100;

    brakeMode = COAST;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}


//TODO: Implement setWheelSpeeds to set the speed of each wheel based on the desired wheel speeds. 
void ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds;

    LF.setSpeed(wheelSpeeds.LF);
    RF.setSpeed(wheelSpeeds.RF);
    LB.setSpeed(wheelSpeeds.LB);
    RB.setSpeed(wheelSpeeds.RB);
}

void ChassisSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode)
{
    double yawCurrent = 0;
    if (mode == YAW_ORIENTED)
    {   //TODO: Figure this out 

        // Consider the following: Updating Yaw Current

        // Account for rollover at 360 degrees

        // remember your rotate ChassisSpeeds
    }
    else if (mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = desiredChassisSpeeds_; // ChassisSpeeds in m/s
    }
    else if (mode == ODOM_ORIENTED) 
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }

        double yawDelta = yawOdom - yawCurrent;
        double imuDelta = imuOdom - imuAngles.yaw;
        double delta = imuDelta - yawDelta;
        double del = yawOdom + delta;
        while (del > 360.0) del -= 360;
        while (del < 0) del += 360;
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawOdom + delta);
    }
    else if (mode == YAW_ALIGN)
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }

        // Compute yaw error(how much the yaw needs to recorrect)
        double yawError = (yawCurrent - yawPhase);
        while (yawError > 180) yawError -= 360;
        while (yawError < -180) yawError += 360;
        
        if (abs(yawError) < 5) yawError = 0;

        if (yawError > 90) yawError -= 180;
        else if (yawError < -90) yawError += 180;

        float yaw_velo = (yawCurrent - yawPrior);
        float deg2rad = PI/180; // convert to rad and just run at 2x that rad/s
        pid_align.feedForward = yaw_velo * yaw_velo_gain;
        float omegaCmd = pid_align.calculatePeriodic(yawError, 1000) * deg2rad;

        if (abs(omegaCmd) < 0.1) omegaCmd = 0;

        ChassisSpeeds xAlignSpeeds = {desiredChassisSpeeds_.vX, desiredChassisSpeeds_.vY, omegaCmd};
        desiredChassisSpeeds = rotateChassisSpeed(xAlignSpeeds, yawCurrent);
    }
    yawPrior = encoder->encoderMovingAverage();
    
    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds); // in m/s (for now)
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
}

ChassisSpeeds ChassisSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent)
{
    // rotate angle counter clockwise
    double theta = (yawCurrent - yawPhase) / 180 * PI;

    return {speeds.vX * cos(theta) - speeds.vY * sin(theta),
            speeds.vX * sin(theta) + speeds.vY * cos(theta),
            speeds.vOmega};
}


WheelSpeeds ChassisSubsystem::normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const
{
    double speeds[4] = {wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB};
    double max_speed = MAX_VEL;

    for (double speed : speeds)
        if (speed > max_speed)
            max_speed = speed;

    if (max_speed > MAX_VEL)
        for (double &speed : speeds)
            speed = speed / max_speed * m_OmniKinematicsLimits.max_Vel;

    return {speeds[0], speeds[1], speeds[2], speeds[3]};
}

