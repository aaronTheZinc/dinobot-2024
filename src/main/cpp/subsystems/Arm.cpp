#include <subsystems/Arm.h>
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace ArmConstants;

Arm::Arm() {
    m_encoder.SetPositionConversionFactor(kEncoderConversionFactor);
    m_pidController.SetP(1);
    m_pidController.SetI(0);
    m_pidController.SetD(0);
    m_pidController.SetFF(0);
    m_pidController.SetOutputRange(-0.5, 0.5);
    m_motor.SetInverted(true);
    m_motor.SetSmartCurrentLimit(kArmCurrentLimit.value());

}

void Arm::setState(ARM_POSITION state) {
    arm_state = state;
}

ARM_POSITION Arm::getState() {
    return arm_state;
}


void Arm::Periodic() {
    double pos = m_encoder.GetPosition();
    double referencePoint = usePosition(arm_state);
    frc::SmartDashboard::PutNumber("Arm: ", pos);

    /**
     * Terminates Pid Control To Allow Manual Movement
    */
    if(arm_state == AMP) {
        return;
    }


    double speed = xPID.Calculate(m_encoder.GetPosition(), referencePoint);
    
    /**
     * Handle Manual Spin down for arm when other button is pressed in teleop.
    */
    if(pos > 0.2) {
        m_motor.Set(0.5);
        return;
    }


    //Set Max Speed & Direction
    bool n  = speed < 0;
    if(std::abs(speed) > 0.6) {
        speed = 0.5;
      if(n) {
            speed = speed * -1;
        };
    };
    
    speed = speed *1.6;
    m_motor.Set(speed);
    
};    


void Arm::move(double speed) {
    m_motor.Set( speed);
}

double Arm::usePosition(ARM_POSITION pos) {
    double m_ref;
        switch(pos) {
        case ARM_POSITION::FLOOR: {
            m_ref = 0.043;
            // m_ref = 0.046;
            break;
        };
        case ARM_POSITION::SPEAKER: {
            m_ref = 0.031;
            break;
        };
        case ARM_POSITION::SHOOT_FAR: {
            m_ref = 0.022508;
            break;
        };
        case ARM_POSITION::Home: {
            m_ref = 0;
            break;
        };
        case ARM_POSITION::WING_SHOT: {
            m_ref = 0.024591;
            break;
        }
        default: {
            m_ref = 0;
            // m_ref = arm_state;
        }
    };

    return m_ref;

};