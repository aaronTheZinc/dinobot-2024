#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
class Intake  : public frc2::SubsystemBase   {
    public:
    Intake();
    rev::CANSparkMax intake_motor{20, rev::CANSparkMax::MotorType::kBrushless};
    bool isReversing = false;
    void start();
    void stop();
    void reverse(double);
    bool hasNote();
    bool didKickback();
    void Periodic() override;

    private:
    frc::DigitalInput beam_break{0};
    bool _hasNote = false;
    frc::Timer kickback_timer;
    double kickback_ref = 0;
    bool _didKickback = false;
};