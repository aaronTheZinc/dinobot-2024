#pragma once
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

class Climber : public frc2::SubsystemBase {
  public: 
  Climber();
  rev::CANSparkMax m_left{50, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right{55, rev::CANSparkMax::MotorType::kBrushless};
  void Periodic() override;
  void toggle();
  void move(double);
  
  bool extended = false;

  private:
    rev::SparkRelativeEncoder m_LeftEncoder = m_left.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController m_LeftPidController =
    m_left.GetPIDController();

    rev::SparkRelativeEncoder m_RightEncoder = m_right.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController m_RightPidController =
    m_right.GetPIDController();

};