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
    ZERO_CLIMB,
    ARM_REVERSE,
    ARM_FORWARD,
    HIGH_UP
};

class Climber {
    private:

        TalonFX *climber_talon1;
        TalonFX *climber_talon2;
        TalonFX *arm_talon1; // 12:74:18:72:18:76 = 6.333333333????
        TalonFX *arm_talon2;

        std::shared_ptr<frc::DoubleSolenoid> m_solenoid;

        const float TICKS_PER_ROTATION = 2048.0;
        float m_arm_gear_ratio = 1 / ((12.0 / 74.0) * (74.0 / 18.0) * (18.0 / 72.0) * (72.0 / 18.0) * (18.0 / 76.0));
        float climb_up_val = 40480.0;
        float climb_down_val = -40480.0;
        float back_arm_angle = -10.0;
        float forward_arm_angle = 50.0;

    public:

    States current_state;
    States last_state;

    Climber();

    float CalculateAngle(float n); 
    void Init();
    void Stop();
    void Up();
    void Down();
    void ArmReverse();
    void ArmForward();
    void Zero();
    void HighUp();
    void climberStateMachine();
    
};
