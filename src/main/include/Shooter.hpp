#pragma once

#include <ctre/Phoenix.h>
#include <frc/DriverStation.h>
#include <memory>

constexpr float reverseSpeed = -0.6f;
constexpr float redShootSpeed = 3675.0f * 2048.0 / 600.0; //3200 close // 3850 far - 3700-3730ish
constexpr float blueShootSpeed = 3800.0f * 2048.0 / 600.0; //3200 close // 3850 far blue 3800
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

    std::shared_ptr<TalonFX> m_motor1;
    std::shared_ptr<TalonFX> m_motor2;

    ShooterState m_State, m_LastState;
};