#include "Drive/DriveConstants.hpp"

#include <ctre/phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/SerialPort.h>
#include <AHRS.h>

class DriveBase{
public:
    DriveBase(frc::Joystick *joy_throttle, frc::Joystick *joy_wheel);
    void Controller();
    void ChecklrLimits();
    void UpdatelVelocity();
    void UpdaterVelocity();

    double target_l = 0, 
        target_r = 0, 
        target_yaw = 0, 
        throttle = 0, 
        wheel = 0,
        curr_max_yaw = 0, 
        curr_yaw = 0,
        curr_l = 0,
        curr_r = 0,
        kF_l = 0,
        kF_r = 0,
        ff_r = 0,
        ff_l = 0;

    double yaw_error = 0;

    double yaw_delta = 0, 
    last_yaw_error = 0;

    double yaw_out;
    
private:
    WPI_TalonFX *m_falcon_left1, *m_falcon_left2, 
        *m_falcon_right1, *m_falcon_right2;

    AHRS *ahrs;

    frc::Joystick *m_joy_throttle;
    frc::Joystick *m_joy_wheel;

};