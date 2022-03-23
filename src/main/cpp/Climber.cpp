#include "Climber.hpp"

Climber::Climber(frc::Joystick* joy) : joyOp(joy)
{
    m_solenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 4, 5);

    climber_talon1 = new TalonFX(30);
    climber_talon2 = new TalonFX(31);
    arm_talon1 = new TalonFX(32);
    arm_talon2 = new TalonFX(33);

    climber_talon1->ConfigFactoryDefault();
    climber_talon2->ConfigFactoryDefault();
    arm_talon1->ConfigFactoryDefault();
    arm_talon2->ConfigFactoryDefault();

    climber_talon1->Config_kP(0, 0.02428 * .8);
    climber_talon1->Config_kI(0, 0.000003);
    climber_talon1->Config_kD(0, 0.00025795 / 100);

    arm_talon1->Config_kP(0, 0.02428);
    arm_talon1->Config_kI(0, 0.0025);
    arm_talon1->Config_kD(0, 0.0004795);

    arm_talon2->Config_kP(0, 0.02428);
    arm_talon2->Config_kI(0, 0.0025);
    arm_talon2->Config_kD(0, 0.0004795);

    climber_talon1->Config_IntegralZone(0, 20000, 50);
    // climber_talon1->ConfigMaxIntegralAccumulator(0, 2048.0 * 3, 0);

    arm_talon1->Config_IntegralZone(0, 1280, 50);

    arm_talon1->SetInverted(true);
    arm_talon2->SetInverted(false);
    climber_talon1->SetInverted(false);
    climber_talon2->SetInverted(false);

    climber_talon1->SetNeutralMode(NeutralMode::Brake);
    climber_talon2->SetNeutralMode(NeutralMode::Brake);

    // climber_talon1->SetNeutralMode(NeutralMode::Coast);
    // climber_talon2->SetNeutralMode(NeutralMode::Coast);

    climber_talon2->Follow(*climber_talon1);
    arm_talon2->Follow(*arm_talon1);
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
    climber_talon2->Set(TalonFXControlMode::PercentOutput, 0);
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
    climber_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
    arm_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Climber::Up()
{
    // climber_talon1->Config_kP(0, 0.0005);
    // climber_talon1->Config_kI(0, 0);
    // climber_talon1->Config_kD(0, 0);
    climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(false, 10, 10, 0.1));
    climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(false, 10, 10, 0.1));
        climber_talon1->Config_kP(0, 0.02428 * .8);
    climber_talon1->Config_kI(0, 0.000003);
    climber_talon1->Config_kD(0, 0.00025795 / 100);
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
    climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 200, 200, 5.0));
    climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 200, 200, 5.0));
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

    // arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(-15.0));
    // arm_talon2->Set(TalonFXControlMode::Position, CalculateAngle(-15.0));
    // arm_talon2->Set(TalonFXControlMode::PercentOutput, 0.0);
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
    if(reach_limit_right && reach_limit_right) {
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

void Climber::climberStateMachine()
{
    // Talon1 Smart Dashboard
    //  frc::SmartDashboard::PutNumber("Talon1 Voltage", climber_talon1->GetMotorOutputVoltage());
     frc::SmartDashboard::PutNumber("Talon1 Percent Out", climber_talon1->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Climb Talon1", climber_talon1->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("Sensor Velocity Talon1", climber_talon1->GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("Sensor Pos Arm Talon1", arm_talon1->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("lArm current", arm_talon1->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("rArm current", arm_talon2->GetOutputCurrent());

    // Talon2 Smart Dashboard
    //  frc::SmartDashboard::PutNumber("Talon2 Voltage", climber_talon2->GetMotorOutputVoltage());
     frc::SmartDashboard::PutNumber("Talon2 Percent Out", climber_talon2->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Climb Talon2", climber_talon2->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("Sensor Velocity Talon2", climber_talon2->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Sensor Pos Arm Talon2", arm_talon2->GetSelectedSensorPosition());

    frc::SmartDashboard::PutNumber("Curr State", (int)current_state);

    
    switch (current_state)
    {
    
    case States::INIT:
        Init();
        current_state = States::ZERO_CLIMB;
        break;
    
    
    case States::STOP_CLIMB:
        // if (last_state != States::STOP_CLIMB) {
        Stop();
        // }
        break;

    case States::UP_CLIMB:
        // if (last_state != States::UP_CLIMB) {
        Up();
        // }
        break;
    case States::DOWN_CLIMB:
        // if (last_state != States::DOWN_CLIMB) {
        Down();
        // }
        break;
    
    /*
    case States::HIGH_UP:
        HighUp();
        break;
    */
    

    case States::ARM_REVERSE:
        ArmReverse();
        break;

    case States::ARM_FORWARD:
        ArmForward();
        break;

    case States::ZERO_CLIMB:
        // if (last_state != States::ZERO_CLIMB) {
        Zero();
        // }
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