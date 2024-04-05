#include "commands/Shooter.h"


ShootCommand::ShootCommand(Intake* _intake, Shooter* _shooter) {
    intake = _intake;
    shooter = _shooter;
}

frc2::Command* ShootCommand::getShootCommand() {
//  auto shoot = frc2::FunctionalCommand(
//   // Reset encoders on command start
//   [this] {  },//init,
//   // Start driving forward at the start of the command
//   [this] { shooter. },//execute,
//   // Stop driving at the end of the command
//   [this] (bool interrupted) {}, //handle interupt},
//   // End the command when the robot's driven distance exceeds the desired value
//   [this] { return false; },
//   // Requires the drive subsystem
//   {intake, &shooter}
// );
//     return shoot;

    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] () {
            // this->intake->reverse(0.4);
    })
    );

}



