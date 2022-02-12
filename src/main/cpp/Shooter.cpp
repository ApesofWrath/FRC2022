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

void Shooter::shoot() { // Use GetAlliance() to change RPM if needed
    m_motor1->Set(ControlMode::Velocity, shootSpeed);
}

void Shooter::intake() {

}

void Shooter::stop() {
    m_motor1->Set(ControlMode::PercentOutput, 0.0f);
}

void Shooter::waiting() {
    m_motor1->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Shooter::reverse() {
    m_motor1->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Shooter::shooterStateMachine() {
    frc::SmartDashboard::PutNumber("alliaance", frc::DriverStation::GetAlliance());
    frc::SmartDashboard::PutNumber("Shooter RPM", sensorUnitsToRPM(m_motor2->GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("Shooter Pos", sensorUnitsToRPM(m_motor2->GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("Shooter percent out", m_motor1->GetMotorOutputPercent());

    switch(m_State) {
        case ShooterState::Init:
            frc::SmartDashboard::PutString("ShooterState", "Init");
            m_LastState = ShooterState::Init;
            m_State = ShooterState::Stop;
        break;
        case ShooterState::Stop:
            frc::SmartDashboard::PutString("ShooterState", "Stop");
            if (m_LastState != ShooterState::Stop) {
                stop();
            }
            m_LastState = ShooterState::Stop;
        break;
        case ShooterState::Shoot:
            frc::SmartDashboard::PutString("ShooterState", "Shoot");
            if (m_LastState != ShooterState::Shoot) {
                shoot();
            }
            m_LastState = ShooterState::Shoot;
        break;
        case ShooterState::Waiting:
            frc::SmartDashboard::PutString("ShooterState", "Waiting");
            if (m_LastState != ShooterState::Waiting) {
                waiting();
            }
            m_LastState = ShooterState::Waiting;
        break;
        case ShooterState::Reverse:
            frc::SmartDashboard::PutString("ShooterState", "Reverse");
            if (m_LastState != ShooterState::Reverse) {
                reverse();
            }
            m_LastState = ShooterState::Reverse;
        break;
    }
}