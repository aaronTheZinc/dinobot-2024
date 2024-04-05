#pragma once

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"


enum RobotCMDS {
    NONE=0,
    SCORE=1,
};

class Dashboard {
    public:
    Dashboard();
    void periodic();
    int getCommand();
    void commandCompleted(int);
    private: 
    int command;
    std::shared_ptr<nt::NetworkTable> dash;
};