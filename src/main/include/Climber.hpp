#pragma once

#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

#include "Constants.hpp"

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

        const float TICKS_PER_ROT = 2048.0;
        const float PI = 3.1415;
        const float m_arm_ratio = 1 / ((12.0 / 74.0) * (74.0 / 18.0) * (18.0 / 72.0) * (72.0 / 18.0) * (18.0 / 76.0));
        const float m_elevator_ratio = (82.0 / 12.0);
        const float m_pulley_diameter = 1.287;
        bool reach_limit_left = false;
        bool reach_limit_right = false;
        const float climb_up = 5.0; // 25 in
        const float climb_slight_up = 1.0; // 5 in 
        const float climb_down = 0.0;
        const float climb_offset = 0.2; // 1 in

        const float back_arm_angle = -10.0;
        const float forward_arm_angle = 50.0;
        const float arm_offset = 1.0;

        int m_sequence_counter = 0;

        frc::Joystick* joyOp;

    public:

    States current_state;
    States last_state;

    Climber(frc::Joystick* joyOp);

    float CalculateAngle(float n); 
    float CalculateHeight(float n);
    void Init();
    void Stop();
    void Up();
    void Down();
    void ArmReverse();
    void ArmForward();
    void Zero();
    void HighUp();

    void climberStateMachine();
    
    void CoastElevator();
    void BrakeElevtor();
};