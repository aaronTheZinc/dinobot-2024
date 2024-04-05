#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
Shooter::Shooter() {
    m_right.SetInverted(true);
    m_left.SetInverted(false);
};

void Shooter::start() {
    _isShooting = true;
    if(start_time == 0) {
    start_time = (double)timer.GetFPGATimestamp();
    };
    
    m_left.Set(1);
    m_right.Set(-1);
};

void Shooter::start(double speed) {
    _isShooting = true;
    if(start_time == 0) {
    start_time = (double)timer.GetFPGATimestamp();
    };
    m_left.Set(speed);
    m_right.Set(-1 * speed);
};


void Shooter::stop() {
    _isShooting = false;
    start_time = 0;
    m_left.Set(0);
    m_right.Set(0);
};

std::tuple<bool, bool, bool, bool> Shooter::useShootSequence(double feedTime=0.35) {
    double currentTime = (double)timer.GetFPGATimestamp() - start_time;
    bool intakeShooterRun = currentTime <= 0.1;
    // bool shooterRun = currentTime >= 0.3;
    bool shooterRun = currentTime >= 0.25;
    bool intakeFeed = currentTime >= feedTime;
    bool completed = currentTime >= 0.45;
    return std::make_tuple(intakeShooterRun, shooterRun, intakeFeed, completed);
};

void Shooter::reverse() {
    _isShooting = true;
    m_left.Set(-0.4);
    m_right.Set(0.4);
}

void Shooter::amp() {
    m_left.Set(0.1);
    m_right.Set(-0.1);
}

bool Shooter::isShooting() {
    return _isShooting;
}