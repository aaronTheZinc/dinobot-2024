// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <iostream>

using namespace DriveConstants;


double sigmoid(double x) {
    return (x / (1 + std::abs(x))) * 0.3;
}


frc::Pose2d b_amp = frc::Pose2d(
            frc::Translation2d(units::length::meter_t(7.584172), units::length::meter_t(7.584172)), 
            frc::Rotation2d(units::angle::radian_t(-1.492656)));

frc::Pose2d b_speaker = frc::Pose2d(
            frc::Translation2d(units::length::meter_t(1.358801), units::length::meter_t(5.33516)), 
            frc::Rotation2d(units::angle::radian_t(-0.070847)));


frc::Pose2d forwardPose = frc::Pose2d(
            frc::Translation2d(units::length::meter_t(1.5), units::length::meter_t(0)), 
            frc::Rotation2d(units::angle::radian_t(0)));

frc::Pose2d startPose = frc::Pose2d(
            frc::Translation2d(units::length::meter_t(0), units::length::meter_t(0)), 
            frc::Rotation2d(units::angle::radian_t(0)));

RobotContainer::RobotContainer() {
    // autoCMD = Auto{&m_shooter, &m_intake, &m_arm};
    limelight = new LL();

    pose.m_drive = &m_drive;
    frc::Field2d m_field;

// Do this in either robot or subsystem init
    frc::SmartDashboard::PutData("Field", &m_field);

// Do this in either robot periodic or subsystem periodic
    m_field.SetRobotPose(pose.getEstimatedPose());
    
    // pose.limelight = &limelight;


    // dashboard = new Dashboard();
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

     pilot.SetDefaultCommand(frc2::RunCommand(
        [this] {

            if(pilot.getInPoseRange()) {
                // m_SysController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.8);
                m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1 );
            } 
            
            if(m_driverController.GetLeftBumper()){
                pilot.enable();
                
                pilot.setProfile(MotionProfile{
                    Auto::LEFT_WING,
                    ARM_POSITION::SHOOT_FAR
                });
            }

            if(m_driverController.GetAButtonPressed()){
                pilot.enable();
                
                pilot.setProfile(MotionProfile{
                    Auto::FAR_CENTER,
                    ARM_POSITION::WING_SHOT
                });
            }



            if(m_driverController.GetRightBumper()){
                pilot.enable();
                
                pilot.setProfile(MotionProfile{
                    Auto::RIGHT_WING,
                    ARM_POSITION::SPEAKER
                });
            }

            if(m_driverController.GetYButton()) {
                pilot.enable();
                
                pilot.setProfile(MotionProfile{
                    b_speaker,
                    ARM_POSITION::SPEAKER
                });
            }
            

            if(m_driverController.GetLeftBumperReleased() || m_driverController.GetRightBumperReleased() || m_driverController.GetYButtonReleased() || m_driverController.GetAButtonReleased()) {
                pilot.disable();
            }
        },{&pilot}));


      m_climber.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // m_climber.move(m_SysController.GetLeftY());
            // m_climber.move(m_SysController.GetLeftY());
            if(m_SysController.GetRightBumperPressed()) {
                m_climber.toggle();
            };


            // if(m_climber.extended) {
            //     m_arm.setState(ARM_POSITION::CLIMB);
            // };
        }
    , {&m_climber}));


    m_intake.SetDefaultCommand(frc2::RunCommand(
        [this] {
            bool isShooting = m_shooter.isShooting();
            bool isAmp = m_arm.getState() == ARM_POSITION::AMP;
            if(m_intake.hasNote() &&  !isShooting && !lockAuto && !isAmp) {
                m_intake.stop();
                m_SysController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.3);
                m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.1 );
                return;
           
            }else {
                m_SysController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
                m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
             
            }
            // if(!beam_break.Get() && (m_SysController.GetRightTriggerAxis() == 0 && !lockAuto && m_arm.getState() != ARM_POSITION::AMP)) {
            //     m_intake.stop();
            //     m_SysController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.3);
            //     m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.1 );
           
            //     return;
            // }else {
            //     m_SysController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
            //     m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
           
            // };
            if(pilot.isEnabled()) return;
            if(lockAuto) return;

            // frc::SmartDashboard::PutBoolean("BEAM BREAK", m_intake.hasNote() );
   


            if(m_SysController.GetLeftTriggerAxis() > 0) {
                m_intake.start();
            }else if(!m_intake.isReversing && !m_SysController.GetLeftBumper() ) {
                m_intake.stop();
            };
              
            
        }
    , {&m_intake}));

    m_shooter.SetDefaultCommand(frc2::RunCommand(
        [this] {

            double shooterFeedTime = 0.35; // seconds

            if(lockAuto || pilot.isEnabled()) return;

            bool shouldLob = m_driverController.GetRightTriggerAxis() > 0;

            // if(shouldLob) shooterFeedTime = 0;
 
            //handle amp
            if(m_SysController.GetLeftBumper()) {
                m_shooter.amp();
                m_intake.start();
                return;
            };

            auto [bothRun, shooterRun, intakeFeed, _] = m_shooter.useShootSequence(shooterFeedTime);
            bool shooterEnabled = m_SysController.GetRightTriggerAxis() > 0;
            
            if(!shooterEnabled) {
                m_shooter.stop();
                if(m_intake.isReversing) m_intake.stop();
                return;
            }; 

              m_shooter.start();
                if(intakeFeed) {
                    m_intake.start();
                    return;
                }
                if(shooterRun) {
                    m_shooter.start();
                    return;
                }
                if(bothRun) {
                    m_intake.reverse(0.4);
                    m_shooter.reverse(); 
                    return;
                };
        }
    , {&m_shooter}));
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    m_arm.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // m_arm.setState(ARM_POSITION::FLOOR);
            // if(std::abs(m_SysController.GetLeftY()) < 0.075) {
            //     m_arm.move(0);
            //     return;
            // };
            // m_arm.move(m_SysController.GetLeftY());
            // m_arm.move(0.4);
        //  m_arm.move(m_driverController.GetLeftY());
        if(m_arm.getState() == ARM_POSITION::AMP && !lockAuto) {
            m_arm.move(m_SysController.GetLeftY() * 0.4);
        }
        if(m_SysController.GetXButtonPressed()) {
            m_arm.setState(ARM_POSITION::Home);
        }
         if(m_SysController.GetYButtonPressed()) {
            m_arm.setState(ARM_POSITION::SPEAKER);
         }
         if(m_SysController.GetBButtonPressed()) {
            m_arm.setState(ARM_POSITION::AMP);
         };

         if(m_SysController.GetAButtonPressed()) {
            m_arm.setState(ARM_POSITION::FLOOR);
         }

        }
    , {&m_arm}));


  m_drive.SetDefaultCommand(frc2::RunCommand(

      [this] {
        
      this->limelight->periodic();

       BotPosition position = this->limelight->getFieldPosition();

        frc::SmartDashboard::PutNumber("[Chassis X]", position.x);
        frc::SmartDashboard::PutNumber("[Chassis Y]", position.y);
    
    //   m_drive.setEstimatedPose(pilot.get);

    double xOffset = this->limelight->getXOffset(0);
    double yInput = m_driverController.GetLeftY();
    double xInput = m_driverController.GetLeftX();
    double rotationInput = m_driverController.GetRightX();
    // RobotCMDS signal = RobotCMDS::NONE;
    // if(m_driverController.GetRightTriggerAxis() > 0) {
    //     if(std::abs(xOffset) > 5) yInput = sigmoid(xOffset);
    //     rotationInput =  m_drive.GetRotateScoreInput();
        
    // };
    if(m_driverController.GetBackButton() && m_driverController.GetStartButton()) {
        m_drive.ZeroHeading();
    };
    if(pilot.pilotEnabled) return;
        m_drive.Drive(
           - units::meters_per_second_t{frc::ApplyDeadband(
                yInput, OIConstants::kDriveDeadband)},
           -units::meters_per_second_t{frc::ApplyDeadband(
                xInput, OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                rotationInput, OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));

        pose.SetDefaultCommand(frc2::RunCommand(
        [this] {
        //  m_arm.move(m_driverController.GetLeftY());
      

        }
    , {&pose}));
}

void RobotContainer::ConfigureButtonBindings() {
    
//   frc2::JoystickButton(&m_driverController,
//                        frc::XboxController::Button::kRightBumper)
//       .WhileTrue(autoCMD.getScoreSpeakerCommand());
}

void RobotContainer::setAutoState(AutoState state) {
    this->autoState  = state;
    autoTimerRef = (double)autoTimer.GetFPGATimestamp();

}

void RobotContainer::RunAuto() {
        //     m_drive.Drive(
        //    - units::meters_per_second_t{0.075},
        //    -units::meters_per_second_t{0.1},
        //     -units::radians_per_second_t{0},
        //     true, true);
    // switch(autoState) {
    //     case AutoState::SHOOT: {
    //         // m_arm.setState(ARM_POSITION::SPEAKER);
    //         if(!autoTimeHasElapsed(3)) {
    //             return;
    //         }
    //         auto [bothRun, shooterRun, intakeFeed] = m_shooter.stopReversing();
    
    //           m_shooter.start();
    //             if(intakeFeed) {
    //             m_intake.start();
    //             if(autoTimeHasElapsed(5)) {
    //              setAutoState(AutoState::DONE);
    //             return;
    //         };
    //                 return;
    //             }
    //             if(shooterRun) {
    //                 m_shooter.start();
    //                 m_intake.stop();
    //                 return;
    //             }
    //             if(bothRun) {
    //                 m_intake.reverse(0.4);
    //                 m_shooter.reverse(); 
    //                 return;
    //             };
    //             break;
    //     };
    //     case AutoState::LEAVE_COMMUNITY: {
    //         pilot.enable();
    //         m_intake.stop();
    //         m_shooter.stop();
    //         pilot.setProfile(MotionProfile{
    //             forwardPose,
    //             ARM_POSITION::FLOOR,
    //          }); 

    //          if(autoTimeHasElapsed(5)) {
    //             setAutoState(AutoState::DONE);
    //          };
    //          break;
    //     };
    //     case AutoState::DONE: {
    //         m_intake.stop();
    //         m_shooter.stop();    
    //         pilot.disable(); 
    //         lockAuto = false;
    //         break;
    //     };

    // }
};

bool RobotContainer::autoTimeHasElapsed(double t) {
    lockAuto = true;
    double current_time = (double)autoTimer.GetFPGATimestamp() - autoTimerRef;
    return t < current_time;
}

void RobotContainer::AutoInit() {
    pilot.setIgnoreVision(true);
    m_drive.ResetOdometry(startPose);
    autoTimer.Restart();
    autoTimerRef = (double)autoTimer.GetFPGATimestamp();
    return;
};

frc2::Command* RobotContainer::getAmpAuto() {
      frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      {},
      // Pass through these two interior waypoints, making an 's' curve path
    //   {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{0_m, 1.5_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());
  
  // no auto
  return new frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] () {
        m_intake.start();
        m_shooter.amp();
    }),
    frc2::WaitCommand(4.0_s),
    frc2::InstantCommand([this]() {
        m_intake.stop();
        m_shooter.stop();
    }),
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive( 0_mps, 0_mps, 0_rad_per_s, false, false); },
          {}));
};


frc2::Command* RobotContainer::getSpeakerAuto() {
   

 frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);
  

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      {},
      // Pass through these two interior waypoints, making an 's' curve path
    //   {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d{2.4_m, 0_m, 0_deg},
      // Pass the config
      config);


    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{2.4_m, 0_m, 0_deg},
      {},
      // Pass through these two interior waypoints, making an 's' curve path
    //   {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return pose.getEstimatedPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


      frc2::SwerveControllerCommand<4> swerveControllerCommand2(
      exampleTrajectory2, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});



  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());
  
//   auto shootCMD =  this->autoCMD.getScoreSpeakerCommand();
  return new frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] () {
        m_arm.move(0.4);
    }),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this] () {
     m_arm.setState(ARM_POSITION::SPEAKER);
    //  m_shooter.start();
    }),
    //START SHOOTING SEQUENCE
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this] () {
        m_shooter.start();
    }),
    frc2::WaitCommand(1.0_s),
    frc2::InstantCommand([this] () {
        m_intake.start();
        // m_shooter.start();
    }),
    frc2::WaitCommand(1_s),
    //END SHOOTING
    //DRIVE TO SECOND NOTE
    frc2::InstantCommand([this]() {
        // m_intake.stop();
        m_shooter.stop();
        m_arm.setState(ARM_POSITION::FLOOR);
    }),
        frc2::InstantCommand([this] () {
        pilot.enable();
        pilot.setProfile(MotionProfile{
        Auto::MIDDLE_NOTE,
        ARM_POSITION::FLOOR,
     });
    }),
    frc2::WaitCommand(2_s),
  //RETURN TO SPEAKER
    frc2::InstantCommand([this]{
        m_intake.stop();
        pilot.enable();
        pilot.setProfile(MotionProfile{
        b_speaker,
        ARM_POSITION::SPEAKER,
     });
    }),
    frc2::WaitCommand(2_s),
    frc2::InstantCommand([this] () {
        m_intake.reverse(0.4);
        // m_shooter.start();
    }),
    frc2::WaitCommand(0.15_s),
    frc2::InstantCommand([this] () {
        m_intake.stop();
        // m_shooter.start();
    }),
    //RESTART SHOOTER
    frc2::InstantCommand([this] () {
        m_shooter.start();
    }),
    frc2::WaitCommand(1.0_s),
    frc2::InstantCommand([this] () {
        m_intake.start();
        // m_intake.stop();
        // m_shooter.start();
    })
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this]{
    //     // m_intake.stop();
    //     // pilot.enable();
    //     m_shooter.stop();
    //     pilot.setProfile(MotionProfile{
    //     Auto::LEFT_NOTE_INTERMEDIATE,
    //     ARM_POSITION::FLOOR,
    //  });
    // }),
    // frc2::WaitCommand(1.5_s),
    // frc2::InstantCommand([this]{
    //     m_shooter.stop();
    //     pilot.setProfile(MotionProfile{
    //     Auto::LEFT_NOTE,
    //     ARM_POSITION::FLOOR,
    //  });
    // }),
    // frc2::WaitCommand(1.5_s),
    //  frc2::InstantCommand([this]{
    //     m_intake.reverse(0.4);
    //     // pilot.enable();
    //     pilot.setProfile(MotionProfile{
    //     b_speaker,
    //     ARM_POSITION::SPEAKER,
    //  });
    //  }),
    // frc2::WaitCommand(0.15_s),
    // frc2::InstantCommand([this] {
    //    m_intake.stop();
    // }),
    //   frc2::WaitCommand(1.85_s),
    //  frc2::InstantCommand([this] () {
    //     m_intake.stop();
    //     m_shooter.start();
    // }),
    // frc2::WaitCommand(1_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.start();
    //     // m_shooter.start();
    // })
    );
    // frc2::WaitCommand(1.8_s),
    // ///////
    // frc2::InstantCommand([this] () {
    //     // m_shooter.start();
    //     m_intake.reverse(0.4);
    //     // m_shooter.start();
    // }),
    // frc2::WaitCommand(0.15_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.stop();
    //     m_shooter.start();
    // }),
    // frc2::WaitCommand(1_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.start();
    //     // m_shooter.start();
    // })
    // //RESTART SHOOTER
    // frc2::InstantCommand([this] () {
    //     m_shooter.start();
    // }),
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.start();
    //     // m_intake.stop();
    //     // m_shooter.start();
    // })

    
    
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this]{
    //     m_shooter.stop();
    //     pilot.enable();
    //     pilot.setProfile(MotionProfile{
    //     Auto::LEFT_NOTE,
    //     ARM_POSITION::FLOOR,
    //  });
    // }),
    // frc2::WaitCommand(3.0_s),
    // frc2::InstantCommand([this]{
    //     m_intake.stop();
    //     pilot.enable();
    //     pilot.setProfile(MotionProfile{
    //     b_speaker,
    //     ARM_POSITION::SPEAKER,
    //  });
    // }),
    // frc2::InstantCommand([this] () {
    //     m_intake.reverse(0.4);
    //     // m_shooter.start();
    // }),
    // frc2::WaitCommand(0.15_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.stop();
    //     // m_shooter.start();
    // }),
    // //RESTART SHOOTER
    // frc2::InstantCommand([this] () {
    //     m_shooter.start();
    // }),
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.start();
    //     // m_shooter.start();
    

//  );
//   auto moveBack = new DriveToPoint(&pilot, frc::Pose2d{});
//   return new frc2::SequentialCommandGroup(
    // frc2::InstantCommand([this] () {
    //  m_arm.setState(ARM_POSITION::SHOOT_FAR);
    // //  m_shooter.start();
    // }),
    // frc2::WaitCommand(4.0_s),
    // frc2::InstantCommand([this] () {
    //     m_shooter.start();
    // }),
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.start();
    //     // m_shooter.start();
    // }),
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this]() {
    //     // m_intake.stop();
    //     m_shooter.stop();
    //     m_arm.setState(ARM_POSITION::FLOOR);
    // }),
//     frc2::WaitCommand(2.0_s),
//     //   std::move(swerveControllerCommand),
//     frc2::WaitCommand(1.0_s),
//     frc2::InstantCommand([this]() {
//         // m_intake.stop();
//         m_intake.stop();
//         m_arm.setState(ARM_POSITION::SPEAKER);
//     }),
//     frc2::WaitCommand(2.5_s),
//     frc2::InstantCommand([this] () {
//         m_intake.stop();
//         m_shooter.start();
//         // m_shooter.start();
//     }),
//     std::move(swerveControllerCommand2),
//     frc2::WaitCommand(1_s),

//     frc2::InstantCommand([this] () {
//         m_intake.start();
//         // m_shooter.start();
//     }),

    
//     frc2::InstantCommand(
//           [this]() { m_drive.Drive( 0_mps, 0_mps, 0_rad_per_s, false, false); },
//           {})


    // frc2::WaitCommand(0.25_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.stop();
    //     m_shooter.start();
    // }),
    // frc2::WaitCommand(0.15_s),
    // frc2::InstantCommand([this] () {
    //     m_intake.start();
    // }),
    // frc2::WaitCommand(0.5_s),
    // frc2::InstantCommand([this]() {
    //     m_intake.stop();
    //     m_shooter.stop();
    // }),
    //   std::move(swerveControllerCommand),

        //   )
        //   ;
}



frc2::Command* RobotContainer::getStraightAuto() {
 frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

      auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      {},
      // Pass through these two interior waypoints, making an 's' curve path
    //   {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{2.5_m, 0_m, 0_deg},
      // Pass the config
      config);

        frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});


        frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});



      return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] {
            m_arm.move(0.4);
        }),
        frc2::WaitCommand(0.5_s),
        frc2::InstantCommand([this] {
            m_arm.setState(ARM_POSITION::FLOOR);
        }),
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
        [this]() { m_drive.Drive( 0_mps, 0_mps, 0_rad_per_s, false, false); },
          {}));
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
    std::string selectedAuto =  frc::SmartDashboard::GetString("Auto Selector", "none");

    if(selectedAuto == "amp") {
        return getAmpAuto();
    }

    if(selectedAuto == "speaker") {
        return getSpeakerAuto();
    }

    if(selectedAuto == "leave_community") {
        return getStraightAuto();
    };

    if(selectedAuto == "wing") {
        return autoCMD.createWingAutoSequence();
    };


    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] {
            m_arm.move(0.4);
        }),
        frc2::WaitCommand(0.5_s),
        frc2::InstantCommand([this] {
            m_arm.setState(ARM_POSITION::FLOOR);
        })
    ) ;
//   // Set up config for trajectory

};


void RobotContainer::disablePilot() {
    pilot.disable();
    m_drive.Drive( 0_mps, 0_mps, 0_rad_per_s, false, false);
}