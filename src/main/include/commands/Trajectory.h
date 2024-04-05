#pragma once
#include <subsystems/DriveSubsystem.h>
#include <frc2/command/SwerveControllerCommand.h>


class Trajectory {
    public: 
    Trajectory(DriveSubsystem*);

    frc2::SwerveControllerCommand<4> makeCommand();

    private:
    DriveSubsystem* drive;
};