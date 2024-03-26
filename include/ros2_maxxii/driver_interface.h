#ifndef DRIVER_INTERFACE_H
#define DRIVER_INTERFACE_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <sstream>
#include "roboteq_constants.h"

#define CTRL_MOTOR_MAX_VALUE 1000
#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

typedef enum Position {LEFT, RIGHT} Position;


double mapRange(double val1, double max1, double max2);
double saturate(double val, double max_val);

std::string getCmdVersion();
std::string mixedModeMotorMove(float throttle, float steering);
std::string issueCommand(std::string commandType, std::string command, std::string args, int waitms, bool isplusminus = false);
std::string issueCommand(std::string commandType, std::string command, int waitms, bool isplusminus = false);


std::string setConfig(int configItem, int index, int value);
std::string setConfig(int configItem, int value);

std::string setCommand(int commandItem, int value); 
std::string setCommand(int commandItem, int value);

std::string getConfig(int configItem, int index);
std::string getConfig(int configItem);

std::string getValue(int operatingItem, int index);

double extractMeasurement( std::string& reading_str, Position position);

std::string extractValueString(std::string msg, std::string command); 

std::string readEncodersCount();
std::string readEncodersSpeed();
std::string readFeedback();
std::string readMotorsCurrent();
std::string readMotorsVoltage();
std::string readFirmware();
std::string readTemperature();
std::string resetEncoder(int id);
std::string sendMotorCmd(int id, double cmd_value);
std::string commandInputOutput(bool set, Position p);

#endif

