#include <subsystems/Climber.h>


Climber::Climber() {
    m_LeftEncoder.SetPositionConversionFactor(1/36);
    m_LeftEncoder.SetPosition(0);
    m_LeftPidController.SetP(1);
    m_LeftPidController.SetI(0);
    m_LeftPidController.SetD(0);
    m_LeftPidController.SetFF(0);
    m_LeftPidController.SetOutputRange(-1, 1);

    m_left.SetInverted(false);
    m_LeftEncoder.SetPosition(0);

    m_RightEncoder.SetPositionConversionFactor(1/36);
    m_RightEncoder.SetPosition(0);
    m_RightPidController.SetP(1);
    m_RightPidController.SetI(0);
    m_RightPidController.SetD(0);
    m_RightPidController.SetFF(0);
    m_RightPidController.SetOutputRange(-1, 1);
    m_right.SetInverted(false);
    m_RightEncoder.SetPosition(0);


};

void Climber::Periodic() {
    if(m_LeftEncoder.GetPosition() > 5) {
        m_LeftPidController.SetReference(1,rev::CANSparkMax::ControlType::kPosition );
        return;
    }
    frc::SmartDashboard::PutNumber("Clmimber Encoder", m_RightEncoder.GetPosition());
    // );
    if(extended) {
        m_LeftPidController.SetReference(ClimberConstants::kMaxArmPosition, rev::CANSparkMax::ControlType::kPosition);
        m_RightPidController.SetReference(ClimberConstants::kMaxArmPosition, rev::CANSparkMax::ControlType::kPosition);
    }else {
        m_LeftPidController.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
        m_RightPidController.SetReference(0, rev::CANSparkMax::ControlType::kPosition);

    };
};

void Climber::toggle() {
    extended = !extended;
}

void Climber::move(double s) {
    // m_left.Set(s);
    m_right.Set(s);
}

