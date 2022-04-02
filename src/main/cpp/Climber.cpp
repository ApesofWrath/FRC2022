#include "Climber.hpp"

Climber::Climber()
{
    m_solenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 4, 5);

    climber_talon1 = std::make_shared<TalonFX>(31);
    climber_talon2 = std::make_shared<TalonFX>(39);
    arm_talon1 = std::make_shared<TalonFX>(32);
    arm_talon2 = std::make_shared<TalonFX>(33);

    climber_talon1->ConfigFactoryDefault();
    climber_talon2->ConfigFactoryDefault();
    arm_talon1->ConfigFactoryDefault();
    arm_talon2->ConfigFactoryDefault();

    configStatusFrames(climber_talon1);
    configStatusFrames(climber_talon2);
    climber_talon2->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255);
    climber_talon2->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    configStatusFrames(arm_talon1);
    configStatusFrames(arm_talon2);
    arm_talon2->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255);
    arm_talon2->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    climber_talon1->Config_kP(0, 0.02428 * .8);
    climber_talon1->Config_kI(0, 0.000003);
    climber_talon1->Config_kD(0, 0.00025795 / 100);

    // arm_talon1->Config_kP(0, 0.02428);
    // arm_talon1->Config_kI(0, 0.0025);
    // arm_talon1->Config_kD(0, 0.0004795);

    // arm_talon2->Config_kP(0, 0.02428);
    // arm_talon2->Config_kI(0, 0.0025);
    // arm_talon2->Config_kD(0, 0.0004795);

    climber_talon1->Config_IntegralZone(0, 20000, 50);
    // climber_talon1->ConfigMaxIntegralAccumulator(0, 2048.0 * 3, 0);

    // arm_talon1->Config_IntegralZone(0, 1280, 50);

    arm_talon1->SetInverted(true);
    arm_talon2->SetInverted(false);
    climber_talon1->SetInverted(false);
    climber_talon2->SetInverted(false);

    climber_talon1->SetNeutralMode(NeutralMode::Brake);
    climber_talon2->SetNeutralMode(NeutralMode::Brake);

    // climber_talon1->SetNeutralMode(NeutralMode::Coast);
    // climber_talon2->SetNeutralMode(NeutralMode::Coast);

    climber_talon2->Follow(*climber_talon1);
    // arm_talon2->Follow(*arm_talon1);
}

float Climber::CalculateAngle(float n)
{
    return (n / 360.0) * m_arm_ratio * TICKS_PER_ROT;
}

float Climber::CalculateHeight(float n) {
    return n * TICKS_PER_ROT * m_elevator_ratio;
}

void Climber::Init()
{
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    // climber_talon2->Set(TalonFXControlMode::PercentOutput, 0);
    // climber_talon2->SetControlFramePeriod(ControlFrame::Control_3_General, 255);
    // climber_talon2->SetControlFramePeriod(ControlFrame::Control_4_Advanced, 255);
    climber_talon2->SetStatusFramePeriod(StatusFrame::Status_1_General_, 255);
    climber_talon2->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    arm_talon2->Set(TalonFXControlMode::PercentOutput, 0);

    climber_talon1->SetSelectedSensorPosition(0.0);
    climber_talon2->SetSelectedSensorPosition(0.0);

    arm_talon1->SetSelectedSensorPosition(0.0);
    arm_talon2->SetSelectedSensorPosition(0.0);

    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

void Climber::Stop()
{
    // climber_talon1->Config_kP(0, 0.0005);
    // climber_talon1->Config_kI(0, 0);
    // climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
    // climber_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
    arm_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Climber::Up()
{
    // climber_talon1->Config_kP(0, 0.0005);
    // climber_talon1->Config_kI(0, 0);
    // climber_talon1->Config_kD(0, 0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // if (m_sequence_counter == 0)
    // {
    //     m_sequence_counter = 1;
    //     climber_talon1->Set(TalonFXControlMode::Position, CalculateHeight(climb_up));
    //     if (climber_talon1->GetSelectedSensorPosition() >= CalculateHeight(climb_up - climb_offset))
    //     {
    //         current_state = States::ARM_REVERSE;
    //     }
    // }
    climber_talon1->Set(TalonFXControlMode::Position, 78000.0);
}

void Climber::HighUp()
{
    climber_talon1->Set(TalonFXControlMode::Position, climb_up);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    if (climber_talon1->GetSelectedSensorPosition() >= climb_up - climb_offset)
    {
        arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(back_arm_angle));
    }
}

void Climber::Down()
{
    
    // climber_talon1->Config_kP(0, 0.0005);
    // climber_talon1->Config_kI(0, 0);
    // climber_talon1->Config_kD(0, 0);
    
    // climber_talon1->Config_kP(0, 0.02428 * 2 * 3);
    // climber_talon1->Config_kI(0, 0.000003);
    // climber_talon1->Config_kD(0, 0.00025795 / 100);
    if(climber_talon1->GetSelectedSensorPosition() > 2000.0) {
        m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
        climber_talon1->Set(TalonFXControlMode::PercentOutput, -0.90);
    } else {
        m_solenoid->Set(frc::DoubleSolenoid::kForward);
        climber_talon1->Set(TalonFXControlMode::PercentOutput, -0.4);
        reach_limit_left = false;
        reach_limit_right = false;
        current_state = States::ARM_FORWARD;
        // current_state = States::STOP_CLIMB;
    }

    /*
    if (m_sequence_counter == 1)
    {
        m_sequence_counter = 2;
        if (arm_talon1->GetSelectedSensorPosition() <= CalculateAngle(back_arm_angle + arm_offset))
        {
            climber_talon1->Set(TalonFXControlMode::Position, CalculateHeight(climb_down));
            m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
        }
    }
    */
//    climber_talon1->Set(TalonFXControlMode::Position, -3000.0);
    
    // arm_talon1->Set(TalonFXControlMode::Position, calculateAngle(15.0));
}

void Climber::ArmReverse()
{
    if(arm_talon1->GetOutputCurrent() > 5.0 || reach_limit_left) {
        arm_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
        reach_limit_left = true;
    } else {
        arm_talon1->Set(TalonFXControlMode::PercentOutput, -0.1);
    }
    if(arm_talon2->GetOutputCurrent() > 5.0 || reach_limit_right) {
        arm_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
        reach_limit_right = true;
    } else {
        arm_talon2->Set(TalonFXControlMode::PercentOutput, -0.1);
    }
    if(reach_limit_left && reach_limit_right) {
        current_state = States::STOP_CLIMB;
    }
}

void Climber::ArmForward()
{
    // if (m_sequence_counter == 2)
    // {
    //     m_sequence_counter = 3;
    //     arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(forward_arm_angle));
    //     if (arm_talon1->GetSelectedSensorPosition() >= CalculateAngle(forward_arm_angle - arm_offset))
    //     {
    //         current_state = States::HIGH_UP;
    //     }
    // }
    // arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(15.0));
    // arm_talon2->Set(TalonFXControlMode::Position, CalculateAngle(15.0));
    if(arm_talon1->GetOutputCurrent() > 5.0 || reach_limit_left) {
        arm_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
        reach_limit_left = true;
    } else {
        arm_talon1->Set(TalonFXControlMode::PercentOutput, 0.1);
    }
    if(arm_talon2->GetOutputCurrent() > 5.0 || reach_limit_right) {
        arm_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
        reach_limit_right = true;
    } else {
        arm_talon2->Set(TalonFXControlMode::PercentOutput, 0.1);
    }
    if(reach_limit_left && reach_limit_right) {
        current_state = States::STOP_CLIMB;
    }
}

void Climber::Zero()
{
    climber_talon1->SetSelectedSensorPosition(0.0);
    climber_talon2->SetSelectedSensorPosition(0.0);
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    arm_talon1->SetSelectedSensorPosition(0.0);
    arm_talon2->SetSelectedSensorPosition(0.0);
    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

void Climber::configStatusFrames(std::shared_ptr<TalonFX> motorController)
{
    std::cout << "TalonFX Motor: " << motorController->GetDeviceID() << "\n";
    std::cout << "Status Frame 1: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 1000) << "\n";
    std::cout << "Status Frame 2: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 1000) << "\n";
    std::cout << "Status Frame 3: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 1000) << "\n";
    std::cout << "Status Frame 4: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 1000) << "\n";
    std::cout << "Status Frame 6: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 1000) << "\n";
    std::cout << "Status Frame 7: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 1000) << "\n";
    std::cout << "Status Frame 8: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 1000) << "\n";
    std::cout << "Status Frame 9: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 1000) << "\n";
    std::cout << "Status Frame 10: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 1000) << "\n";
    std::cout << "Status Frame 11: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 1000) << "\n";
    std::cout << "Status Frame 12: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 1000) << "\n";
    std::cout << "Status Frame 13: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 1000) << "\n";
    std::cout << "Status Frame 14: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 1000) << "\n";
    std::cout << "Status Frame 15: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 1000) << "\n";
    std::cout << "Status Frame 17: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 1000) << std::endl;
    std::cout << "Status Frame Brushless Current: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current) << "\n";
    std::cout << "Status Frame 21: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated) << std::endl;

    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated, 255);
    motorController->SetControlFramePeriod(Control_6_MotProfAddTrajPoint, 255);
}

void Climber::climberStateMachine()
{

    
    // Talon1 Smart Dashboard
     frc::SmartDashboard::PutNumber("Talon1 Voltage", climber_talon1->GetMotorOutputVoltage());
     frc::SmartDashboard::PutNumber("Talon1 Percent Out", climber_talon1->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Climb Talon1", climber_talon1->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon1", climber_talon1->GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("Sensor Pos Arm Talon1", arm_talon1->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("lArm current", arm_talon1->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("rArm current", arm_talon2->GetOutputCurrent());

    // Talon2 Smart Dashboard
     frc::SmartDashboard::PutNumber("Talon2 Voltage", climber_talon2->GetMotorOutputVoltage());
     frc::SmartDashboard::PutNumber("Talon2 Percent Out", climber_talon2->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Climb Talon2", climber_talon2->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon2", climber_talon2->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Sensor Pos Arm Talon2", arm_talon2->GetSelectedSensorPosition());

    frc::SmartDashboard::PutNumber("Curr State", (int)current_state);

    frc::SmartDashboard::PutNumber("Climber State", static_cast<uint32_t>(current_state));


    switch (current_state)
    {
    
    case States::INIT:
        Init();
        current_state = States::ZERO_CLIMB;
        last_state = States::INIT;
        break;
    
    
    case States::STOP_CLIMB:
        // if (last_state != States::STOP_CLIMB) {
        Stop();
        last_state = States::STOP_CLIMB;
        // }
        break;

    case States::UP_CLIMB:
        if (last_state != States::UP_CLIMB) {
            climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(false, 40, 40, 0.5));
            climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(false, 40, 40, 0.5));
            climber_talon1->Config_kP(0, 0.02428 * .8);
            climber_talon1->Config_kI(0, 0.000003);
            climber_talon1->Config_kD(0, 0.00025795 / 100);
        }
        Up();
        last_state = States::UP_CLIMB;
        // }
        break;
    case States::DOWN_CLIMB:
        if (last_state != States::DOWN_CLIMB) {
            climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 140, 140, 5.0));
            climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 140, 140, 5.0));
        }
        Down();
        last_state = States::DOWN_CLIMB;
        // }
        break;
    
    /*
    case States::HIGH_UP:
        HighUp();
        break;
    */
    

    case States::ARM_REVERSE:
        if(last_state != States::ARM_REVERSE) {
            reach_limit_right = false;
            reach_limit_left = false;
        }
        ArmReverse();
        last_state = States::ARM_REVERSE;
        break;

    case States::ARM_FORWARD:
        if(last_state != States::ARM_FORWARD) {
            reach_limit_right = false;
            reach_limit_left = false;
        }
        ArmForward();
        last_state = States::ARM_FORWARD;
        break;

    case States::ZERO_CLIMB:
        if (last_state != States::ZERO_CLIMB) {
            Zero();
        }
        last_state = States::ZERO_CLIMB;
        break;
    }
}

void Climber::CoastElevator() {
    climber_talon1->SetNeutralMode(NeutralMode::Coast);
    climber_talon2->SetNeutralMode(NeutralMode::Coast);
    m_solenoid->Set(frc::DoubleSolenoid::kReverse);
}

void Climber::BrakeElevtor() {
    climber_talon1->SetNeutralMode(NeutralMode::Brake);
    climber_talon2->SetNeutralMode(NeutralMode::Brake);
}
