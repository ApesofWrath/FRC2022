#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/geometry/Pose2d.h>
#include <frc/PowerDistribution.h>
#include <frc/SpeedController.h>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include <units/voltage.h>

#include "Drive/DriveBase.hpp"


    constexpr auto kTicksPerRot = 2048.0;
    constexpr auto kCycleTime = 0.1;
    constexpr auto kWheelDiameter = 6;
    constexpr auto kWheelDiameter_m = kWheelDiameter * 0.0254;
    constexpr auto kWheelCirc = kWheelDiameter_m * 3.14159;
    constexpr auto kTicksPerMeter = 2048.0 * (63.0 / 5.0) / kWheelCirc;
    constexpr auto kMetersPerTick = 1.0 / kTicksPerMeter;

class AutonDrive : public frc2::SubsystemBase {
public:
    AutonDrive(int left1, int left2, int right1, int right2, const std::shared_ptr<AHRS>& ahrs);


    void Periodic() override;
    double getHeading();
    double getAverageEncoderDistance();

    frc::Pose2d getPose();
    frc::DifferentialDriveWheelSpeeds getWheelSpeeds();

    void tankDriveVolts(units::volt_t left, units::volt_t right);

    double percentFromVolts(units::volt_t volts);

    void resetOdometry(frc::Pose2d pose);
    double getTurnRate();
    void setMaxOutput(double max);
    void resetEncoders();

    void configure();

    // double kMetersPerTick = (6.0 *  0.0254) / 2048.0;
    

private:

    WPI_TalonFX left1_drive, left2_drive, right1_drive, right2_drive;

    std::shared_ptr<AHRS> ahrs;
    std::shared_ptr<frc::PowerDistribution> power_distribution;

    frc::SpeedControllerGroup left_motors{left1_drive, left2_drive};
    frc::SpeedControllerGroup right_motors{right1_drive, right2_drive};

    frc::DifferentialDriveOdometry *odometry;
    frc::DifferentialDrive drive{left_motors, right_motors};

};