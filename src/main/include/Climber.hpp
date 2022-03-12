#pragma once

#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>

enum class States {
    INIT,
    STOP_CLIMB, 
    UP_CLIMB, 
    DOWN_CLIMB, 
    ZERO_CLIMB
};

class Climber {
    private:

        TalonFX *climber_talon1;
        TalonFX *climber_talon2;
        TalonFX *arm_talon1; // 12:74:18:72:18:76 = 6.333333333????
        TalonFX *arm_talon2;

        std::shared_ptr<frc::DoubleSolenoid> m_solenoid;

        float m_arm_gear_ratio = 1 / ((12.0 / 74.0) * (74.0 / 18.0) * (18.0 / 72.0) * (72.0 / 18.0) * (18.0 / 76.0));

    public:

    States current_state;
    States last_state;

    Climber();

    void Init();
    void Stop();
    void Up();
    void Down();
    void Zero();
    void climberStateMachine();   
};

