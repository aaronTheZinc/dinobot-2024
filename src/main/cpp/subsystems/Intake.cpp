#include <subsystems/Intake.h>

Intake::Intake() {
 
}
void Intake::start(){
  intake_motor.Set(0.9);  
}
void Intake::stop(){
  isReversing = false;
  intake_motor.Set(0);  
}

void Intake::reverse(double s) {
  isReversing = true;
  intake_motor.Set(-1 * s);
};

bool Intake::hasNote() {
  return !beam_break.Get();
};


void Intake::Periodic() {
  // _hasNote = hasNote();
  // if(hasNote()) {
  //   if(!_hasNote) {
  //     kickback_ref = (double) kickback_timer.GetFPGATimestamp();
  //   }
  //   double timeRelative = (double)kickback_timer.GetFPGATimestamp() - kickback_ref;

  //   if(timeRelative >= 1) {
  //     _didKickback = true;
  //     stop();
  //   }else {
  //     reverse(0.5);
  //   };
  //   _hasNote = true;
  //   return;
  // };
  // _hasNote = false;
  // _didKickback = false;
  // kickback_ref = (double) kickback_timer.GetFPGATimestamp();
};

bool Intake::didKickback() {
  return  _didKickback;
}