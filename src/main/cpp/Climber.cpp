#include "Climber.hpp"

Climber::Climber()
{
    m_solenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 5, 6);

    climber_talon1 = new TalonFX(1);
    climber_talon2 = new TalonFX(2);
    arm_talon1 = new TalonFX(3);
    arm_talon2 = new TalonFX(4);

    climber_talon1->ConfigFactoryDefault();
    climber_talon2->ConfigFactoryDefault();
    arm_talon1->ConfigFactoryDefault();
    arm_talon2->ConfigFactoryDefault();

    climber_talon1->Config_kP(0, 0.02428);
    climber_talon1->Config_kI(0, 0.001);
    climber_talon1->Config_kD(0, 0.00025795);

    arm_talon1->Config_kP(0, 0);
    arm_talon1->Config_kI(0, 0);
    arm_talon1->Config_kD(0, 0);

    climber_talon1->Config_IntegralZone(0, 1024, 50);
    arm_talon1->Config_IntegralZone(0, 0, 0);

    arm_talon1->SetInverted(false);
    arm_talon2->SetInverted(false);
    climber_talon1->SetInverted(false);
    climber_talon2->SetInverted(false);
    
    climber_talon2->Follow(*climber_talon1);
    arm_talon2->Follow(*arm_talon1);
}

float Climber::CalculateAngle(float n) {
    return (n / 360.0) * m_arm_gear_ratio * TICKS_PER_ROTATION;
}

void Climber::Init()
{
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    climber_talon2->Set(TalonFXControlMode::PercentOutput, 0);
    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    arm_talon2->Set(TalonFXControlMode::PercentOutput, 0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

void Climber::Stop()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    
}

void Climber::Up()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, 40480.0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Climber::HighUp()
{
    climber_talon1->Set(TalonFXControlMode::Position, 40480.0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    if (climber_talon1->GetSelectedSensorPosition() >= 40480.0) {
        arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(-15.0));
    }
}

void Climber::Down()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    if (arm_talon1->GetSelectedSensorPosition() >= CalculateAngle(-15.0)) {
        climber_talon1->Set(TalonFXControlMode::Position, -40480.0);
        m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    
    //arm_talon1->Set(TalonFXControlMode::Position, calculateAngle(15.0));
}

void Climber::ArmReverse()
{
    arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(-15.0));
}

void Climber::ArmForward()
{
    arm_talon1->Set(TalonFXControlMode::Position, CalculateAngle(15.0));
}

void Climber::Zero()
{
    climber_talon1->SetSelectedSensorPosition(0.0);
    climber_talon1->Set(TalonFXControlMode::Position, 0);
    arm_talon1->SetSelectedSensorPosition(0.0);
    arm_talon1->Set(TalonFXControlMode::Position, 0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

void Climber::climberStateMachine() 
{
    //Talon1 Smart Dashboard
    // frc::SmartDashboard::PutNumber("Talon1 Voltage", climber_talon1->GetMotorOutputVoltage());
    // frc::SmartDashboard::PutNumber("Talon1 Percent Out", climber_talon1->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Talon1", climber_talon1->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("Sensor Velocity Talon1", climber_talon1->GetSelectedSensorVelocity());
    
    //Talon2 Smart Dashboard
    // frc::SmartDashboard::PutNumber("Talon2 Voltage", climber_talon2->GetMotorOutputVoltage());
    // frc::SmartDashboard::PutNumber("Talon2 Percent Out", climber_talon2->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Talon2", climber_talon2->GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("Sensor Velocity Talon2", climber_talon2->GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("Curr State", (int) current_state);

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
            last_state = States::STOP_CLIMB;
            break;

        case States::UP_CLIMB:
            // if (last_state != States::UP_CLIMB) {
            Up();
            // }
            last_state = States::UP_CLIMB;
            break;
        
        case States::HIGH_UP:
            HighUp();
            last_state = States::HIGH_UP;
            break;

        case States::DOWN_CLIMB:
            // if (last_state != States::DOWN_CLIMB) {
            Down();
            // }
            last_state = States::DOWN_CLIMB;
            break;

        case States::ARM_REVERSE:
            ArmReverse();
            last_state = States::ARM_REVERSE;
            break;

        case States::ARM_FORWARD:
            ArmForward();
            last_state = States::ARM_FORWARD;
            break;            

        case States::ZERO_CLIMB:
            // if (last_state != States::ZERO_CLIMB) {
            Zero();
            // }
            last_state = States::ZERO_CLIMB;
            break;
    }
}