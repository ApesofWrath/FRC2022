#include "Drive/DriveBase.hpp"

DriveBase::DriveBase(frc::Joystick *joy_throttle, frc::Joystick *joy_wheel) {
    m_joy_throttle = joy_throttle;
    m_joy_wheel = joy_wheel;
    
    m_falcon_right1->SetInverted(true);
    m_falcon_right2->SetInverted(true);

    m_falcon_left2->Follow(*m_falcon_left1);
    m_falcon_right2->Follow(*m_falcon_right1);

    m_falcon_right1->ConfigVoltageCompSaturation(12.0);
    m_falcon_right1->EnableVoltageCompensation(true);

    m_falcon_left1->ConfigVoltageCompSaturation(12.0);
    m_falcon_left1->EnableVoltageCompensation(true);

    m_falcon_right2->ConfigVoltageCompSaturation(12.0);
    m_falcon_right2->EnableVoltageCompensation(true);
    
    m_falcon_left2->ConfigVoltageCompSaturation(12.0);
    m_falcon_left2->EnableVoltageCompensation(true);

    m_falcon_right1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 10));
    m_falcon_left1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 10));

    m_falcon_right1->SetNeutralMode(NeutralMode::Brake);
    m_falcon_right2->SetNeutralMode(NeutralMode::Brake);
    m_falcon_left1->SetNeutralMode(NeutralMode::Brake);
    m_falcon_left2->SetNeutralMode(NeutralMode::Brake);

    ahrs = new AHRS(frc::SerialPort::kUSB);
}

void DriveBase::Controller() {
    throttle = m_joy_throttle->GetY();
    wheel = m_joy_wheel->GetX();

    target_l = MAX_Y_RPM * -1.0 * (throttle * throttle); // 
    target_r = target_l;
    target_yaw = MAX_Y_RPM * (wheel * wheel);

    ChecklrLimits();

    curr_yaw = -1.0 * ahrs->GetRate();
    if(curr_yaw > curr_max_yaw) {
        curr_max_yaw = curr_yaw;
    }

    frc::SmartDashboard::PutNumber("target_r", target_r);
	frc::SmartDashboard::PutNumber("target_l", target_l);
    frc::SmartDashboard::PutNumber("max yaw", curr_max_yaw);
    frc::SmartDashboard::PutNumber("current yaw", curr_yaw);

    yaw_error = target_yaw - curr_yaw;
    yaw_delta = 

    target_l = target_l - (target_yaw * (MAX_Y_RPM / MAX_YAW_RATE));
    target_r = target_r - (target_yaw * (MAX_Y_RPM / MAX_YAW_RATE));

    yaw_out = (kP_yaw * yaw_error) + (yaw_delta * last_yaw_error);

    target_r += yaw_out;
    target_l -= yaw_out;

    ChecklrLimits();

    // Setting Feed Forward Values
    if(target_l < 0.0) {
        kF_l = 1 / MAX_Y_RPM_L_BACKWARDS;
    } else {
        kF_l = 1 / MAX_Y_RPM_L_FORWARDS;
    }

    if(target_r < 0.0) {
        kF_r = 1 / MAX_Y_RPM_R_BACKWARDS;
    } else {
        kF_r = 1 / MAX_Y_RPM_R_FORWARDS;
    }

    ff_r = kF_r * target_r;
    ff_l = kF_l * target_l;
    


    last_yaw_error = yaw_error;
}

void DriveBase::ChecklrLimits() {
    if (target_l > MAX_Y_RPM) {
		target_l = MAX_Y_RPM;
	} else if (target_l < -MAX_Y_RPM) {
		target_l = -MAX_Y_RPM;
	}

	if (target_r > MAX_Y_RPM) {
		target_r = MAX_Y_RPM;
	} else if (target_r < -MAX_Y_RPM) {
		target_r = -MAX_Y_RPM;
	}
}

void DriveBase::UpdatelVelocity() {

}

void DriveBase::UpdaterVelocity() {

}