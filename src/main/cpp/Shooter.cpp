#include <Shooter.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
    m_motor1 = std::make_shared<TalonFX>(0);
    m_motor2 = std::make_shared<TalonFX>(1);
    m_motor1->ConfigFactoryDefault();
    m_motor2->ConfigFactoryDefault();
    m_motor1->SetNeutralMode(NeutralMode::Coast);
    m_motor2->SetNeutralMode(NeutralMode::Coast);
    m_motor2->Follow(*m_motor1);
    m_motor1->ConfigNominalOutputReverse(0.0);
    m_motor1->Config_kP(0, 0.086076 * 2, 50); // 9.8429E-05 // 0.086076 * 2
    m_motor1->Config_kF(0, 0.04973929, 50);
    m_motor2->SetInverted(true);
}

void Shooter::Shoot() {
    m_motor1->Set(ControlMode::Velocity, shootSpeed);
}

void Shooter::Stop() {
    m_motor1->Set(ControlMode::PercentOutput, 0.0f);
}

void Shooter::Waiting() {
    m_motor1->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Shooter::Reverse() {
    m_motor1->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Shooter::ShooterStateMachine() {
    frc::SmartDashboard::PutNumber("Shooter RPM", sensorUnitsToRPM(m_motor2->GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("Shooter Pos", sensorUnitsToRPM(m_motor2->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("Shooter percent out", m_motor1->GetMotorOutputPercent());

    switch(m_state) {
        case ShooterState::INIT:
            frc::SmartDashboard::PutString("ShooterState", "Init");
            m_last_state = ShooterState::INIT;
            m_state = ShooterState::STOP;
        break;
        case ShooterState::STOP:
            frc::SmartDashboard::PutString("ShooterState", "Stop");
            if (m_last_state != ShooterState::STOP) {
                Stop();
            }
            m_last_state = ShooterState::STOP;
        break;
        case ShooterState::SHOOT:
            frc::SmartDashboard::PutString("ShooterState", "Shoot");
            if (m_last_state != ShooterState::SHOOT) {
                Shoot();
            }
            m_last_state = ShooterState::SHOOT;
        break;
        case ShooterState::WAITING:
            frc::SmartDashboard::PutString("ShooterState", "Waiting");
            if (m_last_state != ShooterState::WAITING) {
                Waiting();
            }
            m_last_state = ShooterState::WAITING;
        break;
        case ShooterState::REVERSE:
            frc::SmartDashboard::PutString("ShooterState", "Reverse");
            if (m_last_state != ShooterState::REVERSE) {
                Reverse();
            }
            m_last_state = ShooterState::REVERSE;
        break;
    }
}