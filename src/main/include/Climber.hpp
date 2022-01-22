#pragma once

#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

enum class States {
    STOP_CLIMB, 
    UP_CLIMB, 
    DOWN_CLIMB, 
    ZERO_CLIMB
};

class Climber {
    private:

        TalonFX *climber_talon1;
        TalonFX *climber_talon2;

    public:

    States current_state;
    States last_state;

    Climber();

    void Stop();
    void Up();
    void Down();
    void Zero();
    void climberStateMachine();   
};

