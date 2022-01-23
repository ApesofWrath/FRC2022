#include "Drive/DriveConstants.hpp"

#include <ctre/phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/SerialPort.h>
#include <AHRS.h>

class DriveBase{
public:
    DriveBase(frc::Joystick *joy_op);
    void OldController();
    void Controller();

    /**
     * Makes sure that the target left and right RPM don't
     * exceed the maximum velocity
     */
    void OldChecklrLimits();
    void ChecklrLimits();

    double target_l = 0, 
        target_r = 0, 
        target_yaw = 0, 
        throttle = 0, 
        wheel = 0,
        curr_max_yaw_rate = 0, 
        curr_yaw_rate = 0,
        curr_l_p = 0,
        curr_r_p = 0,
        curr_l_v = 0,
        curr_r_v = 0,
        kF_l = 0,
        kF_r = 0;

    double yaw_error = 0;

    double yaw_delta = 0, 
    last_yaw_error = 0,
    l_error = 0,
    r_error = 0,
    last_l_error = 0,
    last_r_error = 0;

    double yaw_out = 0,
        ff_r_out = 0,
        ff_l_out = 0,
        d_l_out = 0,
        d_r_out = 0,
        p_l_out = 0,
        p_r_out = 0,
        total_out_l = 0,
        total_out_r = 0;
    
private:
    WPI_TalonFX *m_falcon_left1, *m_falcon_left2, 
        *m_falcon_right1, *m_falcon_right2;

    AHRS *ahrs;

    frc::Joystick *m_joy_op;

};