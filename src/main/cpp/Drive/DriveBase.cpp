#include "Drive/DriveBase.hpp"

DriveBase::DriveBase(frc::Joystick *joy_op) {
    m_joy_op = joy_op;
       
    m_falcon_left1 = new WPI_TalonFX(1);
    m_falcon_left2 = new WPI_TalonFX(668);
    m_falcon_right1 = new WPI_TalonFX(668);
    m_falcon_right2 = new WPI_TalonFX(668);

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
    throttle = m_joy_op->GetRawAxis(1);
    wheel = m_joy_op->GetRawAxis(4);

    int reverse_throttle;

    if (throttle > 0.0) {
		reverse_throttle = 1;
	} else {
		reverse_throttle = -1;
	}

    int reverse_wheel;

    if (wheel > 0.0) {
		reverse_wheel = 1;
	} else {
		reverse_wheel = -1;
	}

    target_l = MAX_Y_RPM * reverse_throttle * (throttle * throttle); // 
    target_r = target_l;
    target_yaw = MAX_Y_RPM * reverse_wheel * (wheel * wheel);

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
    yaw_delta = yaw_error - last_yaw_error;

    target_l -= (target_yaw * (MAX_Y_RPM / MAX_YAW_RATE));
    target_r += (target_yaw * (MAX_Y_RPM / MAX_YAW_RATE));

    yaw_out = (K_P_YAW * yaw_error) + (yaw_delta * last_yaw_error);

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

    ff_r_out = kF_r * target_r;
    ff_l_out = kF_l * target_l;
    
    curr_l_v = SENSOR_TIMESETP_MINUTE_CONVERSION * TICKS_PER_ROT * m_falcon_left1->GetSelectedSensorVelocity();
    curr_r_v = SENSOR_TIMESETP_MINUTE_CONVERSION * TICKS_PER_ROT * m_falcon_right1->GetSelectedSensorVelocity();

    l_error = target_l - curr_l_v;
    r_error = target_r - curr_r_v;

    p_l_out = K_P_L * l_error;
    p_r_out = K_P_R * r_error;

    d_l_out = K_D_L * (l_error - last_l_error);
    d_r_out = K_D_R * (r_error - last_r_error);

    total_out_l = p_l_out + d_l_out + ff_l_out;
    total_out_r = p_r_out + d_r_out + ff_r_out; 

    if(total_out_l > 1.0) {
        total_out_l = 1.0;
    } else if(total_out_l < -1.0) {
        total_out_l = -1.0;
    }

    if(total_out_r > 1.0) {
        total_out_r = 1.0;
    } else if(total_out_r < -1.0) {
        total_out_r = -1.0;
    }

    m_falcon_left1->Set(total_out_l / 10);
    m_falcon_right1->Set(total_out_r / 10);

    last_yaw_error = yaw_error;
    last_l_error = l_error;
    last_r_error = r_error;

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

