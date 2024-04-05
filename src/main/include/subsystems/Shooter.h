#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/Timer.h>
class Shooter  : public frc2::SubsystemBase   {
    public:
    rev::CANSparkMax m_left{45, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_right{40, rev::CANSparkMax::MotorType::kBrushless};
    void start();
    void start(double);
    void stop();
    std::tuple<bool, bool, bool, bool> useShootSequence(double);
    void reverse();
    void amp();
    bool isShooting();
    Shooter();
    private:
    bool _isShooting;
    
    frc::Timer timer;
    double start_time;

};
