#pragma once

#include <ctre/Phoenix.h>
#include <UnidirectionalTrapezoidalRamp.hpp>
#include <frc/DriverStation.h>
#include <memory>

#include "Hood.hpp"

constexpr float endpoint = 3000.0f;
constexpr float spooling_endpoint = 2800.0f;
constexpr float reverseSpeed = -0.6f;
constexpr float shootSpeed = endpoint * 2048.0 / 600.0; //3200 close // 3675 far - 3700-3730ish <- real
constexpr float waiting_speed = 200.0 * 2048.0 / 600.0;
constexpr float spooling_speed = 1000.0 * 2048.0 / 600.0;
constexpr float shooterGearRatio = 83.0f / 75.0f;
constexpr float shootingSpeedTolerance = 5.0f;

constexpr float rampTime = 1.25f;

constexpr float sensorUnitsToRPM(float su) {
    return su / 2048.00 * 600.0;
}

constexpr float RPM_TO_TICKS = 2048.0 / 600.0;

enum class ShooterState {
    INIT,
    STOP,
    SHOOT,
    WAITING,
    REVERSE,
    SPOOLING
};

constexpr size_t bufferSize = 12;

class Shooter {
public:

    float *rolling;
    float avgCurrentRPM = 0.0f;
    size_t rollingIdx = 0;

    Shooter();
    void Shoot();
    void Intake();
    void Stop();
    void Waiting();
    void Reverse();
    void Spooling();
    void ShooterStateMachine();

    inline void setState(ShooterState state) { m_state = state; };
    inline ShooterState get_state() { return m_state; };

    bool readyToShoot();

    void setIndexerReady(bool ready);

    int loopsCooldown = 0;

private:

    std::shared_ptr<TalonFX> m_motor1;
    std::shared_ptr<TalonFX> m_motor2;
    
    UnidirectionalTrapezoidalRampController m_controller;
    // UnidirectionalTrapezoidalRampController m_controller_high;
    UnidirectionalTrapezoidalRampController m_spooling_controller;

    // std::shared_ptr<Hood> m_hood;

    ShooterState m_state, m_last_state;

    bool m_indexer_ready;
};