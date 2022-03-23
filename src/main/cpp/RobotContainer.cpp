#include "RobotContainer.hpp"
#include <units/length.h>
#include <units/voltage.h>
#include <units/time.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <iostream>
#include <vector>

const float K_TRACK_WIDTH = 0.68533;//0.688975
//constexpr auto K_MAX_ACCEL = 10.77;
constexpr auto K_S = units::volt_t(0.53229);
constexpr auto K_V = 2.8908 * 1_V * 1_s / 1_m;
constexpr auto K_A = 0.11875 * 1_V * 1_s * 1_s / 1_m;
constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

constexpr auto kP_ = 0.00019388;

constexpr auto kMaxForwardVelocity = 3.870452_m / 1_s;

const frc::DifferentialDriveKinematics K_DRIVE_KINEMATICS{
    units::meter_t(K_TRACK_WIDTH)};

frc2::Command* RobotContainer::GetAutonomousCommand() {

    std::cout << "start\n";
    frc::DifferentialDriveVoltageConstraint *autoVoltageConstraint = new frc::DifferentialDriveVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meter>(K_S, K_V, K_A),
        K_DRIVE_KINEMATICS, units::volt_t(10));

    std::cout << "step 2\n";
    // Set up config for trajectory; sets max velocity/acceleraton in very nice sig figs
    frc::TrajectoryConfig *config = new frc::TrajectoryConfig(kMaxForwardVelocity,//2
                                units::meters_per_second_squared_t(2));//5
                                
    frc::TrajectoryConfig *config2 = new frc::TrajectoryConfig(units::meters_per_second_t(1),
                                units::meters_per_second_squared_t(2));

    // Add kinematics to ensure max speed is actually obeyed
    std::cout << "step 3\n";
    config->SetKinematics(K_DRIVE_KINEMATICS);
    // Apply the voltage constraint
    std::cout << "step 4\n";
    config->AddConstraint(*autoVoltageConstraint);

    frc::Pose2d start, end;
    std::vector<frc::Translation2d> points;

    frc::Trajectory trajectory, trajectory1, trajectory2, trajectoryTurn;
    frc2::RamseteCommand *ramseteCommand, *ramseteCommand1, *ramseteCommand2, *ramseteCommandTurn;



    switch (m_autoSelected) {

        case SPIN:

            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                {},
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(-180_deg)),
                *config);
            trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(-180_deg)),
                {},
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                *config);
            ramseteCommand = new frc2::RamseteCommand(
                trajectory, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});

            ramseteCommand2 = new frc2::RamseteCommand(
                trajectory2, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});

            m_drive->resetOdometry(start);

            return new frc2::SequentialCommandGroup(
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { m_drive->tankDriveVolts(0_V, 0_V); }, {}),
                std::move(*ramseteCommand2),
                frc2::InstantCommand([this] { m_drive->tankDriveVolts(0_V, 0_V); }, {}),
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { m_drive->tankDriveVolts(0_V, 0_V); }, {}),
                std::move(*ramseteCommand2),
                frc2::InstantCommand([this] { m_drive->tankDriveVolts(0_V, 0_V); }, {})
            );
        break;

        case CROSS_INIT_LINE:

            start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
            end = frc::Pose2d(2.098_m, 0_m, frc::Rotation2d(0_deg));
            points = {};
            config->SetReversed(true);
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                start,
                points,
                end,
                *config);
            ramseteCommand = new frc2::RamseteCommand(
                trajectory, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});
            
            m_drive->resetOdometry(start);

            return new frc2::SequentialCommandGroup(
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {})
            );
        break;

        case SHOOT_PRELOAD:

            start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
            end = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
            points = {};
            config->SetReversed(false);
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                start,
                points,
                end,
                *config);
            ramseteCommand = new frc2::RamseteCommand(
                trajectory, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});
            
            m_drive->resetOdometry(start);

            return new frc2::SequentialCommandGroup(

                frc2::InstantCommand([this] {
                    m_hood->setState(HoodState::DOWN);
                    m_shooter->setState(ShooterState::SHOOT);
                    m_indexer->m_state = IndexerState::SHOOT;
                }),

                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {})
            );
        break;


        case THREE_BALL:

            //hub -> 1st -> 2nd -> hub
            
            points = {
                frc::Translation2d(-45.074_in, 0_in),
                frc::Translation2d(1.567_in, 101.473_in),
            };
            
            config->SetReversed(true);
            //hub -> origin
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
                {},
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
                *config
            );

            config->SetReversed(false);
            trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
                // get balls
                //frc::Pose2d(0_in, -.1_in, frc::Rotation2d(180_deg))
                //this might pose a problem bc redifining origin direction as backwards
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(180_deg)),
                points,
                //retract intake
                frc::Pose2d(30_in, 0_in, frc::Rotation2d(-45_deg)),
                *config
            );

            trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
                // retract intake here
                frc::Pose2d(30_in, 0_in, frc::Rotation2d(-45_deg)),
                {},
                //End against the side wall of the hub
                frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
                *config
            );

            m_drive->resetOdometry(start);

            ramseteCommand = new frc2::RamseteCommand(
                trajectory, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});
            
            ramseteCommand1 = new frc2::RamseteCommand(
                trajectory1, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});

            ramseteCommand2 = new frc2::RamseteCommand(
                trajectory2, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});

            return new frc2::SequentialCommandGroup(

                frc2::InstantCommand([this] {
                    m_hood->setState(HoodState::DOWN);
                    m_shooter->setState(ShooterState::SHOOT);
                    m_indexer->m_state = IndexerState::SHOOT;
                    m_intake->setState(IntakeState::INDEXING);
                }),

                // frc2::WaitCommand(1_s);,
                
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {}),

                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::GO);
                    m_indexer->SetState(IndexerState::INTAKE);
                }),
                
                std::move(*ramseteCommand1),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop2\n"; 
                }, {}),

                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::STOP);
                    m_indexer->SetState(IndexerState::WAITING);
                }),

                std::move(*ramseteCommand2),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n"; 
                }, {}),

                frc2::InstantCommand([this] {
                    m_shooter->setState(ShooterState::SHOOT);
                    m_indexer->m_state = IndexerState::SHOOT;
                })

            );
        break;

            default:
            std::cout << "df\n";
            return new frc2::InstantCommand([this] { frc::SmartDashboard::PutString("status","you forgot to run auton...");});

        break;
        
    }
}

void RobotContainer::InitAutoChoices() {
    std::cout << "initauto: step1\n";    
    
    m_chooser.SetDefaultOption("Robot", Auto::SPIN);
    m_chooser.AddOption("Robot", Auto::CROSS_INIT_LINE);
    m_chooser.AddOption("Robot", Auto::SHOOT_PRELOAD);
    m_chooser.AddOption("Robot", Auto::THREE_BALL);
    m_chooser.AddOption("Robot", Auto::FIVE_BALL);
}                  






























//congratulations! you found an easter egg!