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

constexpr auto kMaxForwardVelocity = 1.870452_m / 1_s;

const frc::DifferentialDriveKinematics K_DRIVE_KINEMATICS{
    units::meter_t(K_TRACK_WIDTH)};

frc2::Command* RobotContainer::GetAutonomousCommand() {

    std::cout << "start\n";
    frc::DifferentialDriveVoltageConstraint *autoVoltageConstraint = new frc::DifferentialDriveVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meter>(K_S, K_V, K_A),
        K_DRIVE_KINEMATICS, units::volt_t(10));

    std::cout << "step 2\n";
    // Set up config for trajectory; sets max velocity/acceleraton in very nice sig figs
    frc::TrajectoryConfig *config = new frc::TrajectoryConfig(units::meters_per_second_t(3),//2
                                units::meters_per_second_squared_t(6));//5
                                
    // frc::TrajectoryConfig *config2 = new frc::TrajectoryConfig(units::meters_per_second_t(1),
    //                             units::meters_per_second_squared_t(2));

    // Add kinematics to ensure max speed is actually obeyed
    std::cout << "step 3\n";
    config->SetKinematics(K_DRIVE_KINEMATICS);
    // Apply the voltage constraint
    std::cout << "step 4\n";
    config->AddConstraint(*autoVoltageConstraint);

    frc::Pose2d start, end;
    std::vector<frc::Translation2d> points, points1;
 
    frc::Trajectory trajectory, trajectory1, trajectory2, trajectory3, trajectory4, trajectory5;
    frc2::RamseteCommand *ramseteCommand, *ramseteCommand1, *ramseteCommand2, *ramseteCommand3, *ramseteCommand4, *ramseteCommand5;



    switch (m_autoSelected) {

        case CROSS_INIT_LINE:

            start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
            end = frc::Pose2d(2.098_m, 0_m, frc::Rotation2d(0_deg));     //end = frc::Pose2d(2.098_m, 0_m, frc::Rotation2d(0_deg));
            points = {};
            config->SetReversed(false); //true
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                start,
                {},
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
            end = frc::Pose2d(-1.5_m, 0_m, frc::Rotation2d(0_deg));
            points = {};
            config->SetReversed(true); //false
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                start,
                {},
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
                [this](auto left, auto right) { m_drive->tankDriveVolts(-left, -right); },
                {m_drive});
            
            m_drive->resetOdometry(start);

            return new frc2::SequentialCommandGroup(

                frc2::InstantCommand([this] {
                    m_hood->setState(HoodState::DOWN);
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                }),

                frc2::WaitCommand(3_s),

                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {})
            );
        break;



case TWO_BALL:

            //to test: spool speed errors / shooter shooting late error

            //hub -> 1st -> hub
            
            // points = {
            //     frc::Translation2d(-45.074_in, 0_in),
            //     frc::Translation2d(1.567_in, 101.473_in),
            // };
            
            config->SetReversed(false);

            //trolling
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                // get balls
                // make sure account for intake smasheroo
                
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
                {},
                //1st ball
                frc::Pose2d(45.074_in, 0_in, frc::Rotation2d(0_deg)),
                *config
            );

            config->SetReversed(true);

            trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
                
                frc::Pose2d(45.074_in, 0_in, frc::Rotation2d(0_deg)),
                {},
                //retract intake
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
                *config
            );

            config->SetReversed(false);

            trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
                //this might "pose" a problem bc redifining origin direction as backwards
                //can always set prev trjectory to end 180 and start positive here
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(180_deg)),
                {},
                //End against the side wall of the hub
                frc::Pose2d(-41_in, 4.140_in, frc::Rotation2d(157.59_deg)),  //frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
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

                // frc2::InstantCommand([this] {
                //     m_hood->setState(HoodState::DOWN);
                //     m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                //     m_indexer->m_state = IndexerState::SHOOT;
                //     m_intake->setState(IntakeState::INDEXING);
                // }),

                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::GO);
                    m_indexer->SetState(IndexerState::INTAKE);
                }),

                frc2::WaitCommand(1.5_s),
                
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {}),

                frc2::WaitCommand(.5_s),
                
                std::move(*ramseteCommand1),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop2\n"; 
                }, {}),

                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::STOP);
                    m_indexer->SetState(IndexerState::WAITING);
                }),

                frc2::WaitCommand(.5_s),

                std::move(*ramseteCommand2),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n"; 
                }, {}),

                frc2::WaitCommand(1_s),

                frc2::InstantCommand([this] {
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                })

                //wait command needed here to make shooter actually shoot?

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
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                    m_intake->setState(IntakeState::INDEXING);
                }),

                frc2::WaitCommand(3_s),
                
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {}),

                frc2::WaitCommand(.5_s),

                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::GO);
                    m_indexer->SetState(IndexerState::INTAKE);
                }),

                frc2::WaitCommand(.5_s),
                
                std::move(*ramseteCommand1),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop2\n"; 
                }, {}),

                frc2::WaitCommand(2_s),

                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::STOP);
                    m_indexer->SetState(IndexerState::WAITING);
                }),

                frc2::WaitCommand(.5_s),

                std::move(*ramseteCommand2),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n"; 
                }, {}),

                frc2::WaitCommand(1_s),

                frc2::InstantCommand([this] {
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                })

            );
        break;

            case FIVE_BALL:
            //make 5ball shoot from tarmac after the 3rd ball (faster than going to hub) DONE
 
            //hub -> 1st -> 2nd -> hub
           
            points = {
                frc::Translation2d(-45.074_in, 0_in),
                frc::Translation2d(1.567_in, 101.473_in)
            };
 
            //
 
            points1 = {
                //4th and 5th
                frc::Translation2d(0_in, 0_in),
                frc::Translation2d(0_in, 0_in)
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
   
            config->SetReversed(false);
            trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
                // retract intake here
                frc::Pose2d(30_in, 0_in, frc::Rotation2d(-45_deg)),
                {},
                //End against the side wall of the hub
                frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
                *config
            );
 
            /////////////////////////////////////////////////////////////////
            //3 and collect 2 at human player loading
 
            //hub -> origin
            config->SetReversed(true);
            trajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
                frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
                {},
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
                *config
            );
 
            config->SetReversed(false);
            trajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
                // get balls and end against human player station
                frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
                {},
                frc::Pose2d(-17.31_in, 256.52_in, frc::Rotation2d(132.25_deg)),
                *config
            );
 
            config->SetReversed(false);
            trajectory5 = frc::TrajectoryGenerator::GenerateTrajectory(
                // go from human player station
                frc::Pose2d(-17.31_in, 256.52_in, frc::Rotation2d(132.25_deg)),
                {},
                frc::Pose2d(14.75_in, 176.0_in, frc::Rotation2d(67.5_deg)),
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
 
                ramseteCommand3 = new frc2::RamseteCommand(
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
 
                ramseteCommand4 = new frc2::RamseteCommand(
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
 
                ramseteCommand5 = new frc2::RamseteCommand(
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
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                    m_intake->setState(IntakeState::INDEXING);
                }),
 
                frc2::WaitCommand(3_s),
               
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] {
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n";
                }, {}),
 
                frc2::WaitCommand(.5_s),
 
                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::GO);
                    m_indexer->SetState(IndexerState::INTAKE);
                }),
 
                frc2::WaitCommand(.5_s),
               
                std::move(*ramseteCommand1),
                frc2::InstantCommand([this] {
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop2\n";
                }, {}),
 
                frc2::WaitCommand(2_s),
 
                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::STOP);
                    m_indexer->SetState(IndexerState::WAITING);
                }),
 
                frc2::WaitCommand(.5_s),
 
                std::move(*ramseteCommand2),
                frc2::InstantCommand([this] {
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n";
                }, {}),
 
                frc2::WaitCommand(1_s),
 
                frc2::InstantCommand([this] {
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                }),
 
                frc2::WaitCommand(.5_s),
 
                std::move(*ramseteCommand3),
                frc2::InstantCommand([this] {
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n";
                }, {}),
 
                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::GO);
                    m_indexer->SetState(IndexerState::INTAKE);
                }),
 
                frc2::WaitCommand(.5_s),
 
                std::move(*ramseteCommand4),
                frc2::InstantCommand([this] {
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n";
                }, {}),
 
                //human intake here
                frc2::WaitCommand(4_s),
 
 
                frc2::InstantCommand([this] {
                    m_intake->setState(IntakeState::STOP);
                    m_indexer->SetState(IndexerState::WAITING);
                }),
 
                std::move(*ramseteCommand5),
                frc2::InstantCommand([this] {
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop3\n";
                }, {}),
 
                frc2::InstantCommand([this] {
                    m_hood->setState(HoodState::UP);
                    m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
                    m_indexer->m_state = IndexerState::SHOOT;
                    m_intake->setState(IntakeState::INDEXING);
                })
 
            );
        break;

            default:
            std::cout << "df\n";
            return new frc2::InstantCommand([this] { frc::SmartDashboard::PutString("status","you forgot to run auton 💀");});

        break;
        
    }
}

void RobotContainer::InitAutoChoices() {
    std::cout << "initauto: step 1\n";    
    
    m_chooser.SetDefaultOption("CIL", Auto::CROSS_INIT_LINE);
    m_chooser.AddOption("Spin", Auto::SPIN);
    m_chooser.AddOption("Cross Init", Auto::CROSS_INIT_LINE);
    m_chooser.AddOption("Shoot Preload", Auto::SHOOT_PRELOAD);
    m_chooser.AddOption("Two ball", Auto::TWO_BALL);
    m_chooser.AddOption("Three ball", Auto::THREE_BALL);
    m_chooser.AddOption("Five bal", Auto::FIVE_BALL);
}                  






























//congratulations! you found an easter egg!