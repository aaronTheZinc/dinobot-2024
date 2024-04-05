#include <Auto.h>

AutoCommand::AutoCommand(Pilot* _pilot, Shooter* _shooter, Intake* _intake, Arm* _arm, DriveSubsystem* _drive) {
    this->shooter = _shooter;
    this->intake = _intake;
    this->arm = _arm;
    this->drive = _drive;
    this->pilot = _pilot;
};

frc2::SwerveControllerCommand<4> AutoCommand::makeDriveCommand(frc::Trajectory trajectory) {
    frc2::SwerveControllerCommand<4> swerveControllerCommand(
      trajectory, [this]() { return drive->GetPose(); },

      drive->kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { drive->SetModuleStates(moduleStates); },

      {drive});

      return swerveControllerCommand;

};


frc2::SequentialCommandGroup* AutoCommand::createWingAutoSequence() {
    return new frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] () {
        arm->move(0.4);
    }),
    frc2::WaitCommand(1_s),
    frc2::InstantCommand([this] () {
     arm->setState(ARM_POSITION::SPEAKER);
    //  m_shooter.start();
    }),
    //START SHOOTING SEQUENCE
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this] () {
        shooter->start();
    }),
    frc2::WaitCommand(1.0_s),
    frc2::InstantCommand([this] () {
        intake->start();
        // m_shooter.start();
    })
    );
    // return new frc2::SequentialCommandGroup(
    //     frc2::InstantCommand([this] {
    //         arm->move(0.4);
    //      }),
    //      frc2::WaitCommand(1.0_s),
    //     frc2::InstantCommand([this] {
    //         pilot->enable();
    //         intake->start();
    //         this->pilot->setProfile(MotionProfile{
    //             Auto::RIGHT_NOTE_WING,
    //             ARM_POSITION::FLOOR
    //         });
    //     }),
    //      frc2::WaitCommand(2.0_s),
    //     frc2::InstantCommand([this] {
    //         pilot->enable();
    //         intake->start();
    //         this->pilot->setProfile(MotionProfile{
    //             Auto::SPEAKER_SHOT,
    //             ARM_POSITION::FLOOR
    //         });
    //     }),
    //     frc2::InstantCommand([this] () {
    //     intake->reverse(0.4);
    //     // m_shooter.start();
    // }),
    // frc2::WaitCommand(0.15_s),
    // frc2::InstantCommand([this] () {
    //     intake->stop();
    //     // m_shooter.start();
    // }),
    // //RESTART SHOOTER
    // frc2::InstantCommand([this] () {
    //     shooter->start();
    // }),
    // frc2::WaitCommand(1.0_s),
    // frc2::InstantCommand([this] () {
    //     intake->start();
    //     // m_intake.stop();
    //     // m_shooter.start();
    // })

    // );
};





// frc2::SequentialCommandGroup* AutoCommand::getScoreSpeakerCommand() {
//     return new frc2::SequentialCommandGroup(frc2::InstantCommand([this] () {
//      arm->setState(ARM_POSITION::FLOOR);
//      intake->start();
//     }));
//     return new frc2::SequentialCommandGroup(
//     frc2::InstantCommand([this] () {
//      arm->setState(ARM_POSITION::SPEAKER);
//     }),
//     frc2::WaitCommand(3.0_s),
//     frc2::InstantCommand([this] () {
//         intake->reverse(0.4);
//         shooter->reverse();
//     }),
//     frc2::WaitCommand(0.25_s),
//     frc2::InstantCommand([this] () {
//         intake->stop();
//         shooter->start();
//     }),
//     frc2::WaitCommand(0.15_s),
//     frc2::InstantCommand([this] () {
//         intake->start();
//     }),
//     frc2::WaitCommand(0.5_s),
//     frc2::InstantCommand([this]() {
//         intake->stop();
//         shooter->stop();
//     })); 
// }