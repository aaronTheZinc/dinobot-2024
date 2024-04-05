
#pragma once
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <Constants.h>
#include <frc/controller/PIDController.h>

enum ARM_POSITION {
    Home=0,
    AMP=0,
    SOURCE=8,
    SHOOT_FAR=9,
    SPEAKER=8,
    FLOOR=14,
    CLIMB=3,
    WING_SHOT
};

class Arm : public frc2::SubsystemBase {
    public:
    void Periodic() override;
    Arm();

    void setState(ARM_POSITION);
    void move(double);
    ARM_POSITION getState();
    
    private:
    rev::CANSparkMax m_motor{30, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkAbsoluteEncoder m_encoder = m_motor.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle);
    rev::SparkPIDController m_pidController =
    m_motor.GetPIDController();
    ARM_POSITION arm_state = ARM_POSITION::AMP; 
    ARM_POSITION kDefinitePosition;
    frc::PIDController xPID{17,0,0};
    // frc::PIDController xPID{17,2,1};
    double usePosition(ARM_POSITION);
    

};


//amp
//floor
//speaker
//stow