#pragma once
#include <frc2/command/SubsystemBase.h>
#include <Limelight.h>
#include <subsystems/Arm.h>
#include <subsystems/DriveSubsystem.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <Pose.h>
#include <frc/controller/PIDController.h>
#include <Constants.h>
#include <commands/Shooter.h>

struct MotionProfile {
    frc::Pose2d chassis_target;
    ARM_POSITION arm_target;
};

class Pilot: public frc2::SubsystemBase {
    public:
    Pilot(Intake*, Shooter*, Arm*, DriveSubsystem*, Pose* pose);
    void setFieldTarget();
    void Periodic() override;
    void setProfile(MotionProfile);
    void enable();
    void disable();
    void setIgnoreVision(bool);
    bool getInPoseRange();
    bool isEnabled();
    void setTargetPose(frc::Pose2d);
    bool pilotEnabled;

    private:
    Pose* pose;
    Arm* arm;
    Intake* intake; 
    Shooter* shooter;
    bool ignoreVision;
    DriveSubsystem* drive;
    MotionProfile profile;
    ShootCommand shootCMD{intake, shooter};
    frc::PIDController xPID{0.55,0,0};
    frc::PIDController yPID{0.55,0,0};
    frc::PIDController thetaPID{0.8,0,0};
    bool inPoseRange = false;
};