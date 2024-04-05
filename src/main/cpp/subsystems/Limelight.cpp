#include <Limelight.h>
#include <iostream>
#include <frc/apriltag/AprilTagPoseEstimate.h>
#include "frc/ComputerVisionUtil.h"
using namespace frc;

LL::LL() {
     ll = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
     auto alliance  = frc::DriverStation::GetAlliance();
     
     if(alliance == frc::DriverStation::Alliance::kBlue) {
          pose_key = "botpose_wpiblue";
     }else {
          pose_key = "botpose_wpired";
     };
     position = BotPosition{0,0};
};

frc::Pose2d  LL::getBotPose2D() {
     std::vector<double> translation =  ll->GetNumberArray("botpose_targetspace", std::vector<double>(6));

     if(translation.size() < 6) {
          return Pose2d{};
     }


     units::radian_t rZ = units::degree_t{translation.at(5)};

     units::meter_t tX = units::meter_t{translation.at(0)};
     units::meter_t tY = units::meter_t{translation.at(1)};
     

     frc::Translation2d t2d{tX, tY};

     frc::Rotation2d rotation{units::radian_t{rZ}};

     frc::Pose2d pose2d{t2d, rotation};



    return pose2d;

}

void LL::periodic()
 {

     frc::Pose2d pose = this->getBotPose2D();


     double x =  units::inch_t{pose.X()}.value();
     double y = units::inch_t{pose.Y()}.value();
      
      frc::SmartDashboard::PutNumber("[Raw X]", x);


     BotPosition current;
     current.x = std::abs(x);
     current.y = std::abs(y); 

     cache.push_back(current);

     int len = cache.size();
     if(len < 5) return;

     BotPosition average;
     BotPosition cummulative;
     
     for(int i=0; i<len; i++) {
          BotPosition p = cache.at(i);
          cummulative.x += p.x;
          cummulative.y += p.y;
     }

     float avgX =  static_cast<float>(cummulative.x) / len;
     float avgY =  static_cast<float>(cummulative.y) / len;

      frc::SmartDashboard::PutNumber("[Average X]", avgX);
      frc::SmartDashboard::PutNumber("[Average Y]", avgY);
    

     average.x = (double)avgX;
     average.y = (double)avgY;

     position =  average;

     cache = std::vector<BotPosition>();

 }

 BotPosition LL::getFieldPosition() {
     return position;
 }

bool LL::ValidTarget() {
     return true;
}


double LL::getXOffset(int id) {

     //  m_objectInField, m_robotToCamera, m_cameraToObjectEntryRef);

  // Convert robot's pose from Pose3d to Pose2d needed to apply vision
  // measurements.
//   frc::Pose2d visionMeasurement2d = visionMeasurement3d.ToPose2d();

  // Apply vision measurements. For simulation purposes only, we don't input a
  // latency delay -- on a real robot, this must be calculated based either on
  // known latency or timestamps.
//   m_poseEstimator.AddVisionMeasurement(visionMeasurement2d,
//                                        frc::Timer::GetFPGATimestamp());
 return ll->GetNumber("tx",0.0);
}