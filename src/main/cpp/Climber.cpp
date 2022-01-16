#include <iostream>
#include <string>
#include "Climber.hpp"

Climber::Climber()
{
    climber_talon1 = new WPI_TalonFX(0);
    climber_talon2 = new WPI_TalonFX(1);

    climber_talon1->Config_kP(0, 0.1);
    climber_talon1->Config_kI(0, 0.1);
    climber_talon1->Config_kD(0, 0.1);

    climber_talon2->Follow(*climber_talon1);
}

void Climber::Stop()
{
    climber_talon1->Set(ControlMode::Position, 0);
}

void Climber::Up()
{
    climber_talon1->Set(ControlMode::Position, 0);
}

void Climber::Down()
{
    climber_talon1->Set(ControlMode::Position, 0);
}

void Climber::Zero()
{
    climber_talon1->Set(ControlMode::Position, 0);
}

void Climber::climberStateMachine()
{
    switch (current_state)
    {
        case States::STOP_CLIMB:
            if (last_state != States::STOP_CLIMB) {
                Stop();
            }
            last_state = States::STOP_CLIMB;
            break;

        case States::DOWN_CLIMB:
            if (last_state != States::DOWN_CLIMB) {
                Down();
            }
            last_state = States::DOWN_CLIMB;
            break;
            
        case States::UP_CLIMB:
            if (last_state != States::UP_CLIMB) {
                Up();
            }
            last_state = States::UP_CLIMB;
            break;

        case States::ZERO_CLIMB:
            if (last_state != States::ZERO_CLIMB) {
                Zero();
            }
            last_state = States::ZERO_CLIMB;
            break;
    }
}