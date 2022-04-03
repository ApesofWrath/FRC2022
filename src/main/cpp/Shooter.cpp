#include <Shooter.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() : m_controller(endpoint, rampTime), m_spooling_controller(spooling_endpoint, rampTime)
{
    m_motor1 = std::make_shared<TalonFX>(1);
    m_motor2 = std::make_shared<TalonFX>(2);

    m_motor1->ConfigFactoryDefault();
    m_motor2->ConfigFactoryDefault();
    // m_motor1->ConfigClosedloopRamp(0.25);
    // m_motor2->ConfigClosedloopRamp(0.25);
    m_motor1->SetNeutralMode(NeutralMode::Coast);
    m_motor2->SetNeutralMode(NeutralMode::Coast);
    m_motor2->Follow(*m_motor1);
    // m_motor2->SetControlFramePeriod(ControlFrame::Control_3_General, 255);
    // m_motor2->SetControlFramePeriod(ControlFrame::Control_4_Advanced, 255);

    configStatusFrames(m_motor1);
    configStatusFrames(m_motor2);
    m_motor2->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255);
    m_motor2->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    m_motor1->ConfigNominalOutputReverse(0.0);
    m_motor1->Config_kP(0, 0.086076 * 10, 50); // 9.8429E-05 // 0.086076 * 2
    m_motor1->Config_kF(0, 0.04973929 * 3675.0 / 3730.0 * 3675.0 / 3700.0, 50);
    // m_motor2->SetInverted(true);
    // m_motor1->SetInverted(true);

    m_motor1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 160, 160, 0.3));
    m_motor2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 160, 160, 0.3));

    m_motor1->SetInverted(true);
    m_motor2->SetInverted(true);

    rolling = new float[bufferSize];
    memset(rolling, 0, bufferSize * sizeof(float));
}

void Shooter::Shoot()
{
    float currentRPM = sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio;
    m_motor1->Set(ControlMode::Velocity, RPM_TO_TICKS * shooterGearRatio * m_controller.calculateValue(currentRPM));
    // m_motor2->Set(ControlMode::Velocity, RPM_TO_TICKS * shooterGearRatio * m_controller.calculateValue(currentRPM));
}

void Shooter::Spooling()
{
    float currentRPM = sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio;
    m_motor1->Set(ControlMode::Velocity, RPM_TO_TICKS * shooterGearRatio * m_controller.calculateValue(currentRPM));
    // m_motor2->Set(ControlMode::Velocity, RPM_TO_TICKS * shooterGearRatio * m_controller.calculateValue(currentRPM));
}

void Shooter::Stop()
{
    // m_motor1->ConfigClosedloopRamp(0.5);
    // m_motor2->ConfigClosedloopRamp(0.5);
    // m_motor1->SetInverted(false);
    // m_motor2->SetInverted(false);
    // m_motor1->Set(ControlMode::Velocity, waiting_speed);
    m_motor1->Set(TalonFXControlMode::PercentOutput, 0.0);
}

void Shooter::Waiting()
{
    // m_motor1->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Shooter::Reverse()
{
    // m_motor1->Set(ControlMode::PercentOutput, reverseSpeed);
}

bool Shooter::readyToShoot()
{
    // float currentRPM = sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio;
    return std::abs(m_controller.getEndpoint() - avgCurrentRPM) <= shootingSpeedTolerance && loopsCooldown == 0;
}

void Shooter::configStatusFrames(std::shared_ptr<TalonFX> motorController)
{
    std::cout << "TalonFX Motor: " << motorController->GetDeviceID() << "\n";
    std::cout << "Status Frame 1: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 1000) << "\n";
    std::cout << "Status Frame 2: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 1000) << "\n";
    std::cout << "Status Frame 3: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 1000) << "\n";
    std::cout << "Status Frame 4: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 1000) << "\n";
    std::cout << "Status Frame 6: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 1000) << "\n";
    std::cout << "Status Frame 7: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 1000) << "\n";
    std::cout << "Status Frame 8: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 1000) << "\n";
    std::cout << "Status Frame 9: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 1000) << "\n";
    std::cout << "Status Frame 10: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 1000) << "\n";
    std::cout << "Status Frame 11: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 1000) << "\n";
    std::cout << "Status Frame 12: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 1000) << "\n";
    std::cout << "Status Frame 13: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 1000) << "\n";
    std::cout << "Status Frame 14: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 1000) << "\n";
    std::cout << "Status Frame 15: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 1000) << "\n";
    std::cout << "Status Frame 17: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 1000) << std::endl;
    std::cout << "Status Frame Brushless Current: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current) << "\n";
    std::cout << "Status Frame 21: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated) << std::endl;

    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated, 255);
    motorController->SetControlFramePeriod(Control_6_MotProfAddTrajPoint, 255);
}

void Shooter::ShooterStateMachine()
{
    float currentRPM = sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio;
    rolling[rollingIdx] = currentRPM;
    rollingIdx = (rollingIdx + 1) % bufferSize;
    float a = 0.0f;
    for (size_t i = 0; i < bufferSize; i++)
    {
        a += rolling[i];
    }
    avgCurrentRPM = a / static_cast<float>(bufferSize);

    if (loopsCooldown > 0)
        loopsCooldown--;
    // frc::SmartDashboard::PutNumber("loops", loopsCooldown);
    frc::SmartDashboard::PutNumber("Shooter RPM", m_motor1->GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("Shooter Pos", sensorUnitsToRPM(m_motor2->GetSelectedSensorPosition()));
    // frc::SmartDashboard::PutNumber("Shooter percent out", m_motor1->GetMotorOutputPercent());
    // frc::SmartDashboard::PutBoolean("speed?", (m_motor1->GetSelectedSensorVelocity() <= spooling_speed));

    // frc::SmartDashboard::PutNumber("Shooter RPM", currentRPM);
    frc::SmartDashboard::PutNumber("Shooter Avg RPM", avgCurrentRPM);
    frc::SmartDashboard::PutNumber("Shooter Target", m_controller.getEndpoint());

    if ((m_last_state == ShooterState::SHOOT_FARWALL || m_last_state == ShooterState::SHOOT_HUB || m_last_state == ShooterState::SHOOT_LAUNCHPAD) && (m_state != ShooterState::SHOOT_FARWALL && m_state != ShooterState::SHOOT_HUB && m_state != ShooterState::SHOOT_LAUNCHPAD))
    {
        m_controller.exit();
    }

    if ((m_last_state == ShooterState::SPOOLING) && (m_state != ShooterState::SPOOLING))
    {
        m_spooling_controller.exit();
    }

    switch (m_state)
    {
    case ShooterState::INIT:
        // frc::SmartDashboard::PutString("ShooterState", "Init");
        m_last_state = ShooterState::INIT;
        m_state = ShooterState::STOP;
        break;
    case ShooterState::STOP:
        // frc::SmartDashboard::PutString("ShooterState", "Stop");
        if (m_last_state != ShooterState::STOP)
        {
            Stop();
        }
        m_last_state = ShooterState::STOP;
        break;
    case ShooterState::SHOOT_FARWALL:
        frc::SmartDashboard::PutString("ShooterState", "Far Wall");
        if (m_indexer_ready)
        {
            if (m_last_state != ShooterState::SHOOT_FARWALL)
            {
                m_controller.setEndpoint(shootSpeed_FarWall);
                m_controller.enter(sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio);
            }
            Shoot();
            m_last_state = ShooterState::SHOOT_FARWALL;
        }
        break;
    case ShooterState::SHOOT_HUB:
        frc::SmartDashboard::PutString("ShooterState", "Hub");
        if (m_indexer_ready)
        {
            if (m_last_state != ShooterState::SHOOT_HUB)
            {
                m_controller.setEndpoint(shootSpeed_Hub);
                m_controller.enter(sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio);
            }
            Shoot();
            m_last_state = ShooterState::SHOOT_HUB;
        }
        break;
    case ShooterState::SHOOT_LAUNCHPAD:
        frc::SmartDashboard::PutString("ShooterState", "Launchpad");
        if (m_indexer_ready)
        {
            if (m_last_state != ShooterState::SHOOT_LAUNCHPAD)
            {
                m_controller.setEndpoint(shootSpeed_Launchpad);
                m_controller.exit();
                m_controller.enter(sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio);
            }
            Shoot();
            m_last_state = ShooterState::SHOOT_LAUNCHPAD;
        }
        break;
    case ShooterState::WAITING:
        // frc::SmartDashboard::PutString("ShooterState", "Waiting");
        if (m_last_state != ShooterState::WAITING)
        {
            Waiting();
        }
        m_last_state = ShooterState::WAITING;
        break;
    case ShooterState::REVERSE:
        // frc::SmartDashboard::PutString("ShooterState", "Reverse");
        if (m_last_state != ShooterState::REVERSE)
        {
            Reverse();
        }
        m_last_state = ShooterState::REVERSE;
        break;
    case ShooterState::SPOOLING:
        // frc::SmartDashboard::PutString("ShooterState", "Spooling");
        if (m_last_state != ShooterState::SPOOLING)
        {
            m_controller.enter(sensorUnitsToRPM(m_motor1->GetSelectedSensorVelocity()) / shooterGearRatio);
        }
        Spooling();
        break;
    }
}

void Shooter::setIndexerReady(bool ready)
{
    m_indexer_ready = ready;
}