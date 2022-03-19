#include <Shooter.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
    m_motor1 = std::make_shared<TalonFX>(1);
    m_motor2 = std::make_shared<TalonFX>(2);
    m_controller = new UnidirectionalTrapezoidalRampController(endpoint, rampTime);
    m_motor1->ConfigFactoryDefault();
    m_motor2->ConfigFactoryDefault();
    m_motor1->SetNeutralMode(NeutralMode::Coast);
    m_motor2->SetNeutralMode(NeutralMode::Coast);
    m_motor2->Follow(*m_motor1);
    m_motor1->ConfigNominalOutputReverse(0.0);
    m_motor1->Config_kP(0, 0.086076 * 2, 50); // 9.8429E-05 // 0.086076 * 2
    m_motor1->Config_kF(0, 0.04973929, 50);
    // m_motor2->SetInverted(true);
    // m_motor1->SetInverted(true);
}

void Shooter::Shoot() {
    m_motor1->SetInverted(true);
    m_motor2->SetInverted(true);
    float currentRPM = sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity());
    m_motor1->Set(ControlMode::Velocity, RPM_TO_TICKS * m_controller->calculateValue(currentRPM));
    m_motor2->Set(ControlMode::Velocity, RPM_TO_TICKS * m_controller->calculateValue(currentRPM));
}

/**
 * Ramp up to about 300-500 rpm at idle using ClosedLoopRamp
 * When shooting disable ClosedLoopRamp then set shooting RPM
 */

void Shooter::Stop() {
    // m_motor1->ConfigClosedloopRamp(0.5);
    // m_motor2->ConfigClosedloopRamp(0.5);
    // m_motor1->SetInverted(false);
    // m_motor2->SetInverted(false);
    // m_motor1->Set(ControlMode::Velocity, waiting_speed);
    m_motor1->Set(TalonFXControlMode::PercentOutput, 0.0);
}

void Shooter::Waiting() {
    // m_motor1->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Shooter::Reverse() {
    // m_motor1->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Shooter::ShooterStateMachine() {
    frc::SmartDashboard::PutNumber("Shooter RPM", m_motor1->GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Shooter Pos", sensorUnitsToRPM(m_motor2->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("Shooter percent out", m_motor1->GetMotorOutputPercent());
    frc::SmartDashboard::PutBoolean("speed?", (m_motor1->GetSelectedSensorVelocity() <= spooling_speed));

    if((m_last_state == ShooterState::SHOOT) && (m_state != ShooterState::SHOOT)) {
        m_controller->exit();
    }

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
                m_controller->enter(sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()));
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