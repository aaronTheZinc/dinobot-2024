// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <Limelight.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <frc/Joystick.h>
#include <Limelight.h>
#include <subsystems/Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/Arm.h>
#include <subsystems/Shooter.h>
#include <subsystems/Climber.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <Pose.h>
#include <Pilot.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <commands/Shooter.h>
#include <frc2/command/WaitCommand.h>
#include <Auto.h>
#include <memory>
#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

// #include <Dash.h>
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

enum AutoState {
  SHOOT,
  LEAVE_COMMUNITY,
  DONE
};
class RobotContainer {
 public:
  RobotContainer();
  void AutoInit();
  void RunAuto();
  void disablePilot();
  bool lockAuto = true;
  frc2::Command* GetAutonomousCommand();


 private:
  LL* limelight;
  Pilot pilot{&m_intake, &m_shooter,&m_arm,  &m_drive,&pose};
  // frc::DigitalInput beam_break{0};

//  frc::Joystick j{0};
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_SysController{1};

 
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  void setAutoState(AutoState);
  bool autoTimeHasElapsed(double);
  frc::Timer autoTimer;
  double autoTimerRef = 0;
  DriveSubsystem m_drive;
  Arm m_arm;
  Intake m_intake;
  Shooter m_shooter;
  Climber m_climber;
  Pose pose;
  AutoState autoState = AutoState::SHOOT;
  frc2::Command* getStraightAuto();
  frc2::Command* getAmpAuto();
  frc2::Command* getSpeakerAuto();
  AutoCommand autoCMD{&pilot, &m_shooter, &m_intake, &m_arm, &m_drive};
  // frc2::Co
  // Dashboard* dashboard;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
