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

void Climber::Init()
{
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    arm_talon1->Set(TalonFXControlMode::PercentOutput, 0);
}

void Climber::Stop()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, 0.0);
    arm_talon1->Set(TalonFXControlMode::Position, 0);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    
}

void Climber::Up()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, 40480.0);
    arm_talon1->Set(TalonFXControlMode::Position, -307.2);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Climber::Down()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, -40480.0);
    arm_talon1->Set(TalonFXControlMode::Position, 307.2);
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
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
            current_state = States::ZERO;
            break;
    }
}