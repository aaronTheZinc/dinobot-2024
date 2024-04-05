#pragma once
#include <frc2/command/SubsystemBase.h>
#include "frc/geometry/Pose2d.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <Constants.h>
#include <subsystems/DriveSubsystem.h>
#include <Limelight.h>
#include <frc/Timer.h>



class Pose : public frc2::SubsystemBase {
    public:
    Pose();
    frc::Pose2d getEstimatedPose();
    void Periodic() override;
    LL* limelight;
    DriveSubsystem* m_drive;
    private:
    frc::SwerveDrivePoseEstimator<4> pose_estimator;
    bool isValidPose();
    void updateDriveBaseOdometry();
    frc::Timer timer;
    units::second_t getTime();

};