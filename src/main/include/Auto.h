#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/DriveSubsystem.h>
#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/Shooter.h>
#include <subsystems/Intake.h>
#include <subsystems/Arm.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <Pilot.h>
class AutoCommand {
    public:
    AutoCommand(Pilot*, Shooter*, Intake*, Arm*, DriveSubsystem*);
    DriveSubsystem* drive;
    Shooter* shooter;
    Intake* intake;
    Arm* arm;
    // frc2::SequentialCommandGroup* getScoreSpeakerCommand();
    frc2::SequentialCommandGroup* autoSequenceGroup;
    frc2::SequentialCommandGroup* createWingAutoSequence();
    // frc2::SequentialCommandGroup* addCommand(frc2::SequentialCommandGroup*, frc2::Command*);
    private: 
    Pilot* pilot;
    frc2::SwerveControllerCommand<4> makeDriveCommand(frc::Trajectory);
     frc::TrajectoryConfig trajectoryConfig{AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration};

    frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};
};


// class DriveToPoint : public frc2::Command {
// public:
//     DriveToPoint(Pilot*, frc::Pose2d);

//     void Initialize() override;

//     void Execute() override;

//     bool IsFinished() override;

//     private:
//     bool complete = false;
//     Pilot* pilot;
//     frc::Pose2d target; 

// };
