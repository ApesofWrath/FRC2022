#include <iostream>

#include "Climber.h"

Climber::Climber()
{
    climber_talon1 = new WPI_TalonFX(0);
    climber_talon2 = new WPI_TalonFX(1);
}

void Climber::Stop()
{
    climber_talon1->Set(idle_speed);
    climber_talon2->Set(idle_speed);
}

void Climber::Up()
{
    climber_talon1->Set(up_speed);
    climber_talon2->Set(up_speed);
}

void Climber::Down()
{
    climber_talon1->Set(down_speed);
    climber_talon2->Set(down_speed);
}

void Climber::StateMachine()
{
    switch (current_state)
    {
        case States::STOP_CLIMB:
            Stop();
            break;

        case States::DOWN_CLIMB:
            Down();
            break;
            
        case States::UP_CLIMB:
            Up();
            break;
    }
}