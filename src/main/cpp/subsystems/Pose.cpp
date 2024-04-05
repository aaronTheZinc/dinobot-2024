#include <Pose.h>
#include <LL.h>
using namespace frc;


frc::SwerveDriveKinematics<4> PoseEstimatorkDriveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};
frc::SwerveModulePosition init_mod;

const wpi::array<frc::SwerveModulePosition, 4>& module_states = wpi::array<frc::SwerveModulePosition, 4>(init_mod, init_mod, init_mod, init_mod);
const frc::Rotation2d& rotation{};
const frc::Pose2d& init_pose{};

Pose::Pose(): pose_estimator{PoseEstimatorkDriveKinematics,rotation, module_states, init_pose}  {

};

void Pose::Periodic() {
    frc::Pose2d visionMeasurement;
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
      visionMeasurement = LimelightHelpers::toPose2D(LimelightHelpers::getBotpose_wpiBlue());
    }else {
      visionMeasurement = LimelightHelpers::toPose2D(LimelightHelpers::getBotpose_wpiRed());
    }
    // frc::Pose2d visionMeasurement = p;
    pose_estimator.Update(m_drive->GetYaw(), m_drive->GetModuleStates());
    if(LimelightHelpers::getTV() == 1) {
        pose_estimator.AddVisionMeasurement(visionMeasurement, getTime());
    }
    frc::Pose2d estimatded_pose = getEstimatedPose();
    double x = estimatded_pose.X().value();
    double y = estimatded_pose.Y().value();
    frc::SmartDashboard::PutNumber("[Pose Estimator X]", x  );
    frc::SmartDashboard::PutNumber("[Pose Estimator Y]", y);
    frc::SmartDashboard::PutNumber("[Pose Estimator Theta]", (double)estimatded_pose.Rotation().Radians());
}

units::second_t Pose::getTime() {
    return timer.GetFPGATimestamp();
}

frc::Pose2d Pose::getEstimatedPose() {
    return pose_estimator.GetEstimatedPosition();
}