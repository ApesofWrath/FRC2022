#pragma once

#include <ctre/Phoenix.h>
#include <frc/DriverStation.h>
#include <memory>

constexpr float reverseSpeed = -0.6f;
constexpr float shootSpeed = 3675.0f * 2048.0 / 600.0; //3200 close // 3675 far - 3700-3730ish <- real
constexpr float waiting_speed = 200.0f * 2048.0 / 600.0;
constexpr float spooling_speed = 1000.0f * 2048.0 / 600.0;

constexpr float sensorUnitsToRPM(float su) {
    return su / 2048.00 * 600.0;
}

enum class ShooterState {
    INIT,
    STOP,
    SHOOT,
    WAITING,
    REVERSE
};

class Shooter {
public:

    Shooter();
    void Shoot();
    void Intake();
    void Stop();
    void Waiting();
    void Reverse();

    void ShooterStateMachine();

    inline void setState(ShooterState state) { m_state = state; };

private:

    std::shared_ptr<TalonFX> m_motor1;
    std::shared_ptr<TalonFX> m_motor2;

    ShooterState m_state, m_last_state;
};