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
#include "Drive/DriveConstants.hpp"

class AutonDrive : public frc2::SubsystemBase {
public:
    AutonDrive(int left1, int left2, int right1, int right2, AHRS* ahrs);


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

private:

    WPI_TalonFX left1_drive, left2_drive, right1_drive, right2_drive;

    AHRS* ahrs;
    frc::PowerDistribution *power_distribution;

    frc::SpeedControllerGroup left_motors{left1_drive, left2_drive};
    frc::SpeedControllerGroup right_motors{right1_drive, right2_drive};

    frc::DifferentialDriveOdometry *odometry;
    frc::DifferentialDrive drive{left_motors, right_motors};

};