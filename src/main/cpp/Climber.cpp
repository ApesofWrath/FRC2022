#include "Climber.hpp"

Climber::Climber()
{
    climber_talon1 = new WPI_TalonFX(1);
    climber_talon2 = new WPI_TalonFX(2);

    climber_talon1->Config_kP(0, 0.00001);
    climber_talon1->Config_kI(0, 0);
    climber_talon1->Config_kD(0, 0);

    climber_talon2->Follow(*climber_talon1);
}

void Climber::Stop()
{
    climber_talon1->Set(ControlMode::PercentOutput, 0);
}

void Climber::Up()
{
    climber_talon1->Set(ControlMode::PercentOutput, .1);
}

void Climber::Down()
{
    climber_talon1->Set(ControlMode::Velocity, .1);
}

void Climber::Zero()
{
    climber_talon1->Set(ControlMode::Velocity, .1);
}

void Climber::climberStateMachine() 
{

    frc::SmartDashboard::PutNumber("Sensor Pos Talon1", climber_talon1->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon1", climber_talon1->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Sensor Pos Talon2", climber_talon2->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon2", climber_talon2->GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("curr state", (int) current_state);

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