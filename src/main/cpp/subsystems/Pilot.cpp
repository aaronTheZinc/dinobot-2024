#include <Pilot.h>

using namespace PilotConstants;

Pilot::Pilot(Intake* intake, Shooter* shooter,Arm* arm, DriveSubsystem* drive, Pose* pose ) {
    this->arm = arm;
    this->drive = drive;
    this->pose = pose;
    this->intake = intake;
    this->shooter = shooter;

    xPID.SetTolerance(xPIDTolerance);
    xPID.SetTolerance(yPIDTolerance);
    xPID.SetTolerance(tPIDTolerance);
};

void Pilot::setIgnoreVision(bool ignore) {
    this->ignoreVision = ignore;
};

void Pilot::setTargetPose(frc::Pose2d p) {
    this->profile.chassis_target = p;
}

void Pilot::Periodic() {
    if(!pilotEnabled)return;
    

    arm->setState(profile.arm_target);
    frc::Pose2d current_pose =  pose->getEstimatedPose();

    // if(ignoreVision) {
    //     current_pose = drive->GetPose();
    // };

    double kX = xPID.Calculate((double)current_pose.X(), (double)profile.chassis_target.X());
    double kY = yPID.Calculate((double)current_pose.Y(), (double)profile.chassis_target.Y());
    double kTheta = thetaPID.Calculate((double)current_pose.Rotation().Radians(), (double)profile.chassis_target.Rotation().Radians());
    
    if(!inPoseRange) {
    drive->Drive(units::meters_per_second_t{kX}, units::meters_per_second_t{kY},units::radians_per_second_t{kTheta}, false,true );
    }else {
    drive->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0},units::radians_per_second_t{0}, false,true );

    }
    bool xPoseInRange = xPID.AtSetpoint();
    bool yPoseInRange = yPID.AtSetpoint();
    bool tPoseInRange = thetaPID.AtSetpoint();

    this->inPoseRange = (xPoseInRange && yPoseInRange && tPoseInRange);
    // return;
    if(!inPoseRange) {
                shooter->stop();
                if(intake->isReversing) intake->stop();
                return;
    };
    if(inPoseRange) {
    auto [bothRun, shooterRun, intakeFeed, _] = shooter->useShootSequence(0.35);
   

              shooter->start();
                if(intakeFeed) {
                    intake->start();
                    return;
                }
                if(shooterRun) {
                    shooter->start();
                    return;
                }
                if(bothRun) {
                    intake->reverse(0.4);
                    shooter->reverse(); 
                    return;
                };
    }

    // if()
    // drive->Drive();

};

void Pilot::setProfile(MotionProfile profile) {
    this->profile = profile;
    return;
};

bool Pilot::getInPoseRange() {
    return this->inPoseRange;
}



void Pilot::enable() {
    pilotEnabled = true;
}

void Pilot::disable() {
    pilotEnabled = false;
}

bool Pilot::isEnabled() {
    return pilotEnabled;
}