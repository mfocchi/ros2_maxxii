#include "motor.h"


Motor::Motor(ctrl_mode_t mode, double torque_constant)
{
    torque_constant_ = torque_constant;
    mode_ = mode;
    current_ = 0.0;
    current_sp_ = 0.0;

    velocity_rpm_ = 0.0;
    velocity_sp_rpm_ = 0.0;
}

Motor::Motor()
{
    Motor(VELOCITY, 0.001);
}

// SETTERS -----------------------------------------------------

void Motor::setActualVelocity(double velocity, vel_unit_t unit)
{
    velocity_rpm_ = convert_vel_into_rpm(velocity, unit);
}

void Motor::setDesiredVelocity(double velocity, vel_unit_t unit)
{
    if(isTorqueMode())
    {
        throw std::logic_error("Error: cannot control velocity when in Troque Control Mode.\n");
    }
    velocity_sp_rpm_ = convert_vel_into_rpm(velocity, unit);
}


void Motor::setActualTorque(double tau)
{   
    double amp = tau * torque_constant;
    setActualCurrent(amp);
}

void Motor::setDesiredTorque(double tau)
{   
    double amp = tau * torque_constant;
    setDesiredCurrent(amp);
}

void Motor::setActualCurrent(double amp)
{
    current_ = amp;
}

void Motor::setDesiredCurrent(double amp)
{
    if(isVelocityMode())
    {
        throw std::logic_error("Error: cannot set desired torque/current when in Velocity Control Mode.\n");
    }
    current_sp_ = amp;
}

// GETTERS ---------------------------------------------

double Motor::getAcutalVelocity(vel_unit_t unit) const
{
    return convert_vel_from_rpm(velocity_rpm_, unit);
}

double Motor::getDesiredVelocity(vel_unit_t unit) const
{
    return convert_vel_from_rpm(velocity_rpm_, unit);
}

double Motor::getActualTorque() const
{
    return current_ / torque_constant_;
}

double Motor::getDesiredTorque() const
{
    return current_sp_ / torque_constant_;
}

double Motor::getActualCurrent() const
{
    return current_;
}

double Motor::getDesiredCurrent() const
{
    return current_sp_;
}

double convert_vel_into_rpm(double value, vel_unit_t unit)
{
    double res = 0.0
    switch (unit)
    {
    case RADPS:
        res = RADPS_TO_RPM * value;
        break;
    case RPM:
        res = value;
    case DEGPS:
        res = DEGPS_TO_RPM * value;
    default:
        throw std::invalid_argument("Error: a non standard measurement unit for velocity was selected for conversion into RPM!\n");
    }
    return res;
}

double convert_vel_from_rpm(double value, vel_unit_t unit)
{
    double res = 0.0
    switch (unit)
    {
    case RADPS:
        res = value / RADPS_TO_RPM;
        break;
    case RPM:
        res = value;
    case DEGPS:
        res = value / DEGPS_TO_RPM;
    default:
        throw std::invalid_argument("Error: a non standard measurement unit for velocity was selected for conversion into RPM!\n");
    }
    return res;
}