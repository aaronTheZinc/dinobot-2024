#pragma once
#include <vector>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation3d.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
#include <frc/DriverStation.h>
#include <string>

struct BotPosition {
    double x;
    double y;
};

class LL {
    public:
    LL();
    bool ValidTarget();
    double getXOffset(int);
    frc::Pose2d getBotPose2D();
    BotPosition getFieldPosition();
    void periodic();
    private:
    std::vector<BotPosition> cache;
    std::shared_ptr<nt::NetworkTable> ll;
    BotPosition position;
    std::string pose_key;

};