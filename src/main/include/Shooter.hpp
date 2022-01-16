#pragma once

#include <ctre/Phoenix.h>
#include <memory>

constexpr float reverseSpeed = -1.f;
constexpr float shootSpeed = 1.f;
constexpr float waitingSpeed = .35f;

constexpr float sensorUnitsToRPM(float su) {
    return su / 2048.0 * 600.0;
}

enum class ShooterState {
    Init,
    Stop,
    Shoot,
    Waiting,
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

    inline void setState(ShooterState state) { m_State = state; };

private:

    std::shared_ptr<TalonFX> m_Motor;

    ShooterState m_State, m_LastState;
};