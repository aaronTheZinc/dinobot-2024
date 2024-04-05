#pragma once
#include <frc2/command/Command.h>
#include <subsystems/Shooter.h>
#include <subsystems/Intake.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
// Define your command class
class ShootCommand  {
    public:
    ShootCommand(Intake*, Shooter*);
    frc2::Command* getShootCommand();
    Intake* intake;
    Shooter* shooter;
};
