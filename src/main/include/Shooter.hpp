#pragma once

enum class ShooterState {
    Init,
    Stop,
    Shoot,
    Reverse
};

class Shooter {
public:

    Shooter();
    void shoot();
    void intake();
    void stop();
    void waiting();
    void reverse();

    void shooterStateMachine();

private:

    ShooterState m_State;    
};