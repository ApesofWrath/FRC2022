#include "Drive/DriveBase.hpp"

#define LOG_V(var) frc::SmartDashboard::PutNumber(#var , var)

DriveBase::DriveBase(frc::Joystick *joy_op) {
     m_joy_op = joy_op;
       
    m_falcon_left1 = new WPI_TalonFX(10);
    m_falcon_left2 = new WPI_TalonFX(12);
    m_falcon_right1 = new WPI_TalonFX(11);
    m_falcon_right2 = new WPI_TalonFX(13);

    m_falcon_left1->ConfigFactoryDefault();
    m_falcon_right1->ConfigFactoryDefault();
    m_falcon_left2->ConfigFactoryDefault();
    m_falcon_right2->ConfigFactoryDefault();

    m_falcon_right1->SetInverted(true);
    m_falcon_right2->SetInverted(true);

    m_falcon_left2->Follow(*m_falcon_left1);
    m_falcon_right2->Follow(*m_falcon_right1);

    m_falcon_left1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20, 20, 0.1));
    m_falcon_left2->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20, 20, 0.1));
    m_falcon_right1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20, 20, 0.1));
    m_falcon_right2->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20, 20, 0.1));

    m_falcon_left1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 33, 33, 0.1));
    m_falcon_left2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 33, 33, 0.1));
    m_falcon_right1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 33, 33, 0.1));
    m_falcon_right2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 33, 33, 0.1));


    m_falcon_left1->ConfigOpenloopRamp(0.0, 0);
    m_falcon_left2->ConfigOpenloopRamp(0.0, 0);
    m_falcon_right1->ConfigOpenloopRamp(0.0, 0);
    m_falcon_right2->ConfigOpenloopRamp(0.0, 0);

    m_falcon_right1->ConfigVoltageCompSaturation(12.0);
    m_falcon_right1->EnableVoltageCompensation(true);

    m_falcon_left1->ConfigVoltageCompSaturation(12.0);
    m_falcon_left1->EnableVoltageCompensation(true);

    m_falcon_right2->ConfigVoltageCompSaturation(12.0);
    m_falcon_right2->EnableVoltageCompensation(true);
    
    m_falcon_left2->ConfigVoltageCompSaturation(12.0);
    m_falcon_left2->EnableVoltageCompensation(true);

    SetCoastNeutral();

 

    ahrs = new AHRS(frc::SerialPort::kMXP);
}

void DriveBase::Controller() {
    throttle = m_joy_op->GetRawAxis(1);
    wheel = -m_joy_op->GetRawAxis(4);

    double reverse_throttle;

    // directional multipliers
    if (throttle > 0.0) {
		reverse_throttle = 1;
	} else {
		reverse_throttle = -1;
	}

    double reverse_wheel;

    if (wheel > 0.0) {
		reverse_wheel = -1;
	} else {
		reverse_wheel = 1;
	}
    LOG_V(throttle * throttle * throttle);
    LOG_V(wheel * wheel * wheel * wheel);
    target_l = MAX_Y_RPM * reverse_throttle * (throttle * throttle * throttle * throttle);
    target_r = target_l;
    target_yaw = MAX_YAW_RATE * reverse_wheel * (wheel * wheel * wheel * wheel);

    ChecklrLimits();

    curr_yaw_rate = -ahrs->GetRate();

    frc::SmartDashboard::PutNumber("target_r before", target_r);
	frc::SmartDashboard::PutNumber("target_l before", target_l);
    frc::SmartDashboard::PutNumber("current yaw", curr_yaw_rate);

    LOG_V(target_yaw);

    target_l -= (target_yaw * MAX_Y_RPM / MAX_YAW_RATE);
    target_r += (target_yaw * MAX_Y_RPM / MAX_YAW_RATE);

    frc::SmartDashboard::PutNumber("target_l after", target_l);
    frc::SmartDashboard::PutNumber("target_r after", target_r);

    yaw_error = target_yaw - curr_yaw_rate;
	if (std::abs(yaw_error) < .3) { //TODO: maybe get rid of this
		yaw_error = 0.0;
	}

    yaw_out = (K_P_YAW * yaw_error);
    LOG_V(yaw_out);


    target_l -= yaw_out;
    target_r += yaw_out;

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
    
    curr_l_v = SENSOR_TIMESTEP_MINUTE_CONVERSION * (m_falcon_left1->GetSelectedSensorVelocity() / TICKS_PER_ROT);
    curr_r_v = SENSOR_TIMESTEP_MINUTE_CONVERSION * (m_falcon_right1->GetSelectedSensorVelocity() / TICKS_PER_ROT);

    l_error = target_l - curr_l_v;
    r_error = target_r - curr_r_v;

    LOG_V(l_error);
    LOG_V(r_error);

    p_l_out = K_P_L * l_error;
    p_r_out = K_P_R * r_error;

    d_l_out = K_D_L * (l_error - last_l_error);
    d_r_out = K_D_R * (r_error - last_r_error);

    LOG_V(p_l_out);
    LOG_V(p_r_out);
    LOG_V(d_l_out);
    LOG_V(d_r_out);
    LOG_V(ff_l_out);
    LOG_V(ff_r_out);

    total_out_l = p_l_out + ff_l_out;
    total_out_r = p_r_out + ff_r_out;

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

    LOG_V(total_out_l);
    LOG_V(total_out_r);

    // total_out_l *= 0.25;
    // total_out_r *= 0.25;
    if(m_joy_op->GetRawButton(6)) {
        kOutputPercent = 0.5;
    } else {
        kOutputPercent = 1.0;
    }

    m_falcon_left1->Set(TalonFXControlMode::PercentOutput, total_out_l * kOutputPercent);
    m_falcon_right1->Set(TalonFXControlMode::PercentOutput, total_out_r * kOutputPercent);

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

void DriveBase::SetBrakeNeutral() {
    m_falcon_right1->SetNeutralMode(NeutralMode::Brake);
    m_falcon_right2->SetNeutralMode(NeutralMode::Brake);
    m_falcon_left1->SetNeutralMode(NeutralMode::Brake);
    m_falcon_left2->SetNeutralMode(NeutralMode::Brake);
}

void DriveBase::SetCoastNeutral() {
    m_falcon_right1->SetNeutralMode(NeutralMode::Coast);
    m_falcon_right2->SetNeutralMode(NeutralMode::Coast);
    m_falcon_left1->SetNeutralMode(NeutralMode::Coast);
    m_falcon_left2->SetNeutralMode(NeutralMode::Coast);
}