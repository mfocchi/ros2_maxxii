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



namespace mdc2460
{
    std::string encodeCmd(std::string cmdType, std::string cmd, std::string args);
    std::string encodeCmd(std::string cmdType, std::string cmd);

    std::string setConfig(int configItem, int index, int value);
    std::string setConfig(int configItem, int value);

    std::string setCommand(int cmdItem, int index, int value);
    std::string setCommand(int cmdItem, int value); 
    std::string setCommand(int cmdItem);

    
    std::string getConfig(int configItem, int index);
    std::string getConfig(int configItem);

    std::string getValue(int operatingItem, int index);

    std::string extractValueString(std::string msg, std::string cmd); 
    bool isItemOutRange(int item);
    int computeIndexFromPosition(Position p);

    std::string reqEncodersCount();
    std::string reqEncodersSpeed();
    std::string reqFeedback();
    std::string reqClosedLoopError();
    std::string reqMotorsCurrent();
    std::string reqMotorsVoltage();
    std::string reqFirmware();
    std::string reqTemperature();

    std::string cmdEmergencyStop();
    std::string resetEncoder(Position p);
    std::string cmdMotor(Position p, double cmd_value);
    std::string cmdMotorRPM(Position p, double cmd_value_rpm);
    std::string cmdInputOutput(Position p, bool set);

    std::string cmdChangeBaudrate(unsigned long baudrate);

    double extractValueDouble(const std::string& reading_str, Position position);
    long extractValueLong(const std::string& reading_str, Position position);     
    std::string removeHeaderMessage(const std::string& reading_str);

    double mapRange(double val1, double max1, double max2);
    void sleep_ms(unsigned long ms);
};

#endif

