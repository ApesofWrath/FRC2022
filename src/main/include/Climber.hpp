#pragma once

#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

enum class States {
    INIT,
    STOP_CLIMB, 
    UP_CLIMB, 
    DOWN_CLIMB,
    DOWN_SLOW_CLIMB,
    ZERO_CLIMB
};

class Climber {
private:
    const float TICKS_PER_ROT = 2048.0;
    const float PI = 3.1415;
    const float m_elevator_ratio = (82.0 / 12.0);
    const float m_pulley_diameter = 1.287;
    bool reach_limit_left = false;
    bool reach_limit_right = false;
    const float climb_up = 5.0; // 25 in
    const float climb_slight_up = 1.0; // 5 in 
    const float climb_down = 0.0;
    const float climb_offset = 0.2; // 1 in
    static constexpr float kElevatorThreshold = 0.5f;

    int m_sequence_counter = 0;

public:

    std::shared_ptr<TalonFX> climber_talon1;
    std::shared_ptr<TalonFX> climber_talon2;

    States current_state = States::INIT;
    States last_state;

    Climber();

    float CalculateHeight(float n);
    void Init();
    void Stop();
    void Up();
    void Down();
    void DownSlow();
    void Zero();
    void configStatusFrames(std::shared_ptr<TalonFX> motorController);
    void climberStateMachine();
    
    void CoastElevator();
    void BrakeElevtor();
};
