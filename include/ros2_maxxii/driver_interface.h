#ifndef DRIVER_INTERFACE_H
#define DRIVER_INTERFACE_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <sstream>
#include <unistd.h> 
#include "roboteq_constants.h"

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

typedef enum Position {LEFT, RIGHT} Position;




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

double extractValueDouble( std::string& reading_str, Position position);
long extractValueLong( std::string& reading_str, Position position);

std::string extractValueString(std::string msg, std::string command); 

std::string readEncodersCount();
std::string readEncodersSpeed();
std::string readFeedback();
std::string readMotorsCurrent();
std::string readMotorsVoltage();
std::string readFirmware();
std::string readTemperature();
std::string resetEncoder(Position p);
std::string sendMotorCmd(Position p, double cmd_value);
std::string commandInputOutput(Position p, bool set);
bool isItemOutRange(int item);

double mapRange(double val1, double max1, double max2);
double saturate(double val, double max_val);
void sleep_ms(unsigned long ms);

#endif

