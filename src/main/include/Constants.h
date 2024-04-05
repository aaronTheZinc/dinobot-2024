// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <numbers>
#include <string>
#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

constexpr units::meter_t kTrackWidth = 0.2032_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.3556_m;



// Chassis configuration
// constexpr units::meter_t kTrackWidth =
//     0.6604_m;  // Distance between centers of right and left wheels on robot
// constexpr units::meter_t kWheelBase =
//     0.6604_m;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;


constexpr int kFrontLeftDrivingCanId = 12;
constexpr int kRearLeftDrivingCanId = 17;
constexpr int kFrontRightDrivingCanId = 13;
constexpr int kRearRightDrivingCanId = 16;

constexpr int kFrontLeftTurningCanId = 11;
constexpr int kRearLeftTurningCanId = 18;
constexpr int kFrontRightTurningCanId = 14;
constexpr int kRearRightTurningCanId = 15;
}  // namespace DriveConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 12;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 1.5_mps;
constexpr auto kMaxAcceleration = 1.5_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.1;
}  // namespace OIConstants

namespace ArmConstants {
    constexpr double kEncoderConversionFactor = (M_PI * 24) / 320;
    const double rangeMin=0;
    const double rangeMax=0;
    const int kAmpHeight = 10;
    const int kSpeakerHeight = 20;
    const int kStowHeight = 0;
    const auto kArmCurrentLimit = 60_A;
    const double maxArmEncoderValue = 0.24;
    
}

namespace BlueFieldConstants {
    const frc::Pose2d P_AMP{};
    const frc::Pose2d P_SPEAKER{};
    const frc::Pose2d P_SOURCE{};
}

namespace RedFieldConstants {
    const frc::Pose2d P_AMP{};
    const frc::Pose2d P_SPEAKER{};
    const frc::Pose2d P_SOURCE{};
};

namespace ClimberConstants {
  constexpr double kEncoderConversionFactor = 1 / 17;
  const double kMaxArmPosition = -107.501801;
};

namespace Auto {
    const frc::Pose2d SPEAKER_SHOT = frc::Pose2d(
            frc::Translation2d(units::length::meter_t(1.358801), units::length::meter_t(5.33516)), 
            frc::Rotation2d(units::angle::radian_t(-0.070847)));
     const std::string autoNames[] = {"Speaker 1"};
     const frc::Pose2d MIDDLE_NOTE =  frc::Pose2d(
            frc::Translation2d(units::length::meter_t(3), units::length::meter_t(5.3)), 
            frc::Rotation2d(units::angle::radian_t(0)));

     const frc::Pose2d RIGHT_NOTE =  frc::Pose2d(
            frc::Translation2d(units::length::meter_t(2.75), units::length::meter_t(6.75)), 
            frc::Rotation2d(units::angle::radian_t(0.376713)));

    const frc::Pose2d LEFT_NOTE_INTERMEDIATE = frc::Pose2d(
            frc::Translation2d(units::length::meter_t(1.75), units::length::meter_t(5.33516)), 
            frc::Rotation2d(units::angle::radian_t(-0.070847)));
    
     const frc::Pose2d LEFT_NOTE =  frc::Pose2d(
            frc::Translation2d(units::length::meter_t(3), units::length::meter_t(7)), 
            frc::Rotation2d(units::angle::radian_t(0.764816)));
    

    const frc::Pose2d FAR_CENTER =  frc::Pose2d(
            frc::Translation2d(units::length::meter_t(2.313146), units::length::meter_t(5.516407)), 
            frc::Rotation2d(units::angle::radian_t(0.000)));

    const frc::Pose2d LEFT_WING =  frc::Pose2d(
            frc::Translation2d(units::length::meter_t(1.798543), units::length::meter_t(7.318973)), 
            frc::Rotation2d(units::angle::radian_t(0.847800)));
    const frc::Pose2d RIGHT_WING =  frc::Pose2d(
        frc::Translation2d(units::length::meter_t(0.874993), units::length::meter_t(4.231817)), 
        frc::Rotation2d(units::angle::radian_t(-1.050510)));

    const frc::Pose2d LEFT_NOTE_WING =  frc::Pose2d(
        frc::Translation2d(units::length::meter_t(2.5), units::length::meter_t(6.94)), 
        frc::Rotation2d(units::angle::radian_t(-0.028187)));   

    const frc::Pose2d RIGHT_NOTE_WING =  frc::Pose2d(
        frc::Translation2d(units::length::meter_t(2.5), units::length::meter_t(4.17)), 
        frc::Rotation2d(units::angle::radian_t(-0.085412)));   

};

namespace PilotConstants {
    const double xPIDTolerance = 0.05; 
    const double yPIDTolerance = 0.1; 
    const double tPIDTolerance = 0.0075; 
}