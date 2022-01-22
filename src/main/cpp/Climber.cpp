#include "Climber.hpp"

Climber::Climber()
{
    climber_talon1 = new TalonFX(1);
    climber_talon2 = new TalonFX(2);

    climber_talon1->ConfigFactoryDefault();
    climber_talon1->Config_kP(0, 0.005);
    climber_talon1->Config_kI(0, 0.0);
    climber_talon1->Config_kD(0, 0.0001);

    climber_talon2->Follow(*climber_talon1);
}

void Climber::Stop()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, 0.0);
    
}

void Climber::Up()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, 20480.0);
}

void Climber::Down()
{
    //climber_talon1->Config_kP(0, 0.0005);
    //climber_talon1->Config_kI(0, 0);
    //climber_talon1->Config_kD(0, 0);
    climber_talon1->Set(TalonFXControlMode::Position, -20480.0);
}

void Climber::Zero()
{
    climber_talon1->SetSelectedSensorPosition(0.0);
    climber_talon1->Set(TalonFXControlMode::Position, 0);

}

void Climber::climberStateMachine() 
{
    //Talon1 Smart Dashboard
    frc::SmartDashboard::PutNumber("Talon1 Voltage", climber_talon1->GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Talon1 Percent Out", climber_talon1->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Talon1", climber_talon1->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon1", climber_talon1->GetSelectedSensorVelocity());
    
    //Talon2 Smart Dashboard
    frc::SmartDashboard::PutNumber("Talon2 Voltage", climber_talon2->GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Talon2 Percent Out", climber_talon2->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Talon2", climber_talon2->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon2", climber_talon2->GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("Curr State", (int) current_state);

    switch (current_state)
    {
        case States::STOP_CLIMB:
            // if (last_state != States::STOP_CLIMB) {
                Stop();
            // }
            last_state = States::STOP_CLIMB;
            break;

        case States::DOWN_CLIMB:
            // if (last_state != States::DOWN_CLIMB) {
                Down();
            // }
            last_state = States::DOWN_CLIMB;
            break;
            
        case States::UP_CLIMB:
            // if (last_state != States::UP_CLIMB) {
                Up();
            // }
            last_state = States::UP_CLIMB;
            break;

        case States::ZERO_CLIMB:
            // if (last_state != States::ZERO_CLIMB) {
                Zero();
            // }
            last_state = States::ZERO_CLIMB;
            break;
    }
}