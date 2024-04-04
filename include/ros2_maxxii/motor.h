#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>

#define RADPS_TO_RPM (30.0 / M_PI)
#define DEGPS_TO_RPM (1.0/6.0)

typedef enum ctrl_mode_t {VELOCITY, TORQUE} ctrl_mode_t;
typedef enum vel_unit_t {RADPS, RPM, DEGPS} vel_unit_t;

double convert_vel_into_rpm(double value, vel_unit_t unit);
double convert_vel_from_rpm(double value, vel_unit_t unit);


class Motor
{
private:
    double torque_constant_;
    double current_;
    double current_sp_;
    double velocity_rpm_;
    double velocity_sp_rpm_;
    ctrl_mode_t mode_;

public:
    Motor(ctrl_mode_t mode, double torque_constant);
    Motor();
    
    void setActualVelocity(double velocity, vel_unit_t unit);
    void setDesiredVelocity(double velocity, vel_unit_t unit);
    void setActualTorque(double tau);
    void setDesiredTorque(double tau);
    void setActualCurrent(double amp);
    void setDesiredCurrent(double amp);

    double getAcutalVelocity(vel_unit_t unit) const;
    double getDesiredVelocity(vel_unit_t unit) const;
    double getActualTorque() const;
    double getDesiredTorque() const;
    double getActualCurrent() const;
    double getDesiredCurrent() const;

    bool isVelocityMode() {return mode_ == VELOCITY;}
    bool isTorqueMode() {return mode_ == TORQUE;}
};


#endif