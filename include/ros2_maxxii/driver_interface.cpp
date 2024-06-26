#include "driver_interface.h"


std::string mdc2460::encodeCmd(std::string cmdType, std::string cmd, std::string args) {
    std::stringstream ss;
    if (args == "")
        ss << cmdType << cmd << "\r";
    else
        ss << cmdType << cmd << " " << args << "\r";
    return ss.str();
}

std::string mdc2460::encodeCmd(std::string cmdType, std::string cmd) {
    return encodeCmd(cmdType, cmd, "");
}

std::string mdc2460::setConfig(int configItem, int index, int value) 
{
    if (isItemOutRange(configItem))
    {
        std::stringstream ss;
        ss << "Cannot set configuration, item out of reange [0,255] = " << configItem << std::endl;
        throw std::runtime_error(ss.str());
    }
    char cmd[10];
    char args[50];
    sprintf(cmd, "$%02X", configItem);
    sprintf(args, "%i %i", index, value);
    if (index == MISSING_VALUE) 
    {
        sprintf(args, "%i", value);
        index = 0;
    }

    if (index < 0)
    {
        std::stringstream ss;
        ss << "Cannot set configuration, index out of range (< 0) = " << index << std::endl;
        throw std::runtime_error(ss.str());
    }

    return encodeCmd("^", cmd, args);
}

std::string mdc2460::setConfig(int configItem, int value) 
{
    return setConfig(configItem, MISSING_VALUE, value);
}

std::string mdc2460::setCommand(int cmdItem, int index, int value) {
    if (isItemOutRange(cmdItem))
    {
        std::stringstream ss;
        ss << "Cannot set command, item out of reange [0,255] = " << cmdItem << std::endl;
        throw std::runtime_error(ss.str());
    }
    char cmd[10];
    char args[50];
    sprintf(cmd, "$%02X", cmdItem);
    sprintf(args, "%i %i", index, value);


    if (index == MISSING_VALUE) {
        if (value != MISSING_VALUE)
            sprintf(args, "%i", value);
        index = 0;
    }

    if (index < 0)
    {
        std::stringstream ss;
        ss << "Cannot set command, index out of range (< 0) = " << index << std::endl;
        throw std::runtime_error(ss.str());
    }
    return encodeCmd("!", cmd, args);
}

std::string mdc2460::setCommand(int cmdItem, int value) 
{
    return setCommand(cmdItem, MISSING_VALUE, value);
}

std::string mdc2460::setCommand(int cmdItem) 
{
    return setCommand(cmdItem, MISSING_VALUE, MISSING_VALUE);
}

std::string mdc2460::getConfig(int configItem, int index)
{
    if (isItemOutRange(configItem))
    {
        std::stringstream ss;
        ss << "Cannot get configuration, item out of reange [0,255] = " << configItem << std::endl;
        throw std::runtime_error(ss.str());
    }
    if (index < 0)
    {
        std::stringstream ss;
        ss << "Cannot get configuration, index out of range (< 0) = " << index << std::endl;
        throw std::runtime_error(ss.str());
    }
    char cmd[10];
    char args[50];

    sprintf(cmd, "$%02X", configItem);
    sprintf(args, "%i", index);

    return encodeCmd("~", cmd, args);
}

std::string mdc2460::getConfig(int configItem) 
{
    return getConfig(configItem, 0);
}

std::string mdc2460::getValue(int operatingItem, int index) 
{
    if (isItemOutRange(operatingItem))
    {
        std::stringstream ss;
        ss << "Cannot get value, operating item out of reange [0,255] = " << operatingItem << std::endl;
        throw std::runtime_error(ss.str());
    }
    if (index < 0)
    {
        std::stringstream ss;
        ss << "Cannot get value, index out of range (< 0) = " << index << std::endl;
        throw std::runtime_error(ss.str());
    }
    char cmd[10];
    char args[50];

    sprintf(cmd, "$%02X", operatingItem);
    sprintf(args, "%i", index);

    return encodeCmd("?", cmd, args);
}

bool mdc2460::isItemOutRange(int item)
{
    return item < 0 || item > 255;
}

std::string mdc2460::removeHeaderMessage(const std::string& msg)
{
    std::string::size_type pos = msg.rfind("=")+1;
    std::string::size_type carriage = msg.find("\r", pos);
    return msg.substr(pos, carriage - pos);
}

double mdc2460::extractValueDouble(const std::string& msg, Position position) 
{
    size_t colonPos = msg.find(':');
    std::string reading;
    if (position == LEFT) 
    {
        reading = msg.substr(0, colonPos);
    }
    else if (position == RIGHT) 
    {
        reading = msg.substr(colonPos + 1);
    }
    return std::stoi(reading);
}

long mdc2460::extractValueLong(const std::string& msg, Position position) 
{ 
    size_t colonPos = msg.find(':');
    std::string reading;
    if (position == LEFT) 
    {
        reading = msg.substr(0, colonPos);
    }
    else if (position == RIGHT) 
    {
        reading = msg.substr(colonPos + 1);
    }
    return std::stol(reading);
}

int mdc2460::computeIndexFromPosition(Position p)
{
    if(p == LEFT)
        return 1;
    else if(p == RIGHT)
        return 2;
    else
        return 0;
}



std::string mdc2460::reqEncodersCount()
{
    return getValue(_C, 0);
}

std::string mdc2460::reqEncodersSpeed()
{
    return getValue(_S, 0);
}

std::string mdc2460::reqFeedback()
{
    return getValue(_F, 0);
}

std::string mdc2460::reqClosedLoopError()
{
    return getValue(_E, 0);
}


std::string mdc2460::reqMotorsCurrent()
{
    return getValue(_A, 0);
}

std::string mdc2460::reqMotorsVoltage()
{
    return getValue(_V, 0);
}

std::string mdc2460::reqFirmware()
{
    return getValue(_FID, 0);
}

std::string mdc2460::reqTemperature()
{
    return getValue(_T, 0);
}

std::string mdc2460::cmdEmergencyStop()
{
    return setCommand(_EX, 1);
}

std::string mdc2460::resetEncoder(Position p)
{
    int idx = computeIndexFromPosition(p);
    return setCommand(_C, idx, 0);
}

std::string mdc2460::cmdMotor(Position p, double cmd_value)
{
    int idx = computeIndexFromPosition(p);
    return setCommand(_GO, idx, cmd_value);
}

std::string mdc2460::cmdMotorRPM(Position p, double cmd_value_rpm)
{
    int idx = computeIndexFromPosition(p);
    return setCommand(_S, idx, cmd_value_rpm);
}

std::string mdc2460::cmdInputOutput(Position p, bool set) 
{
    // 1  Setting D1 OUT
    // 0  Resetting D1 OUT 
    int id = computeIndexFromPosition(p);
    if (set) 
    {
        return setCommand(_D1, id);
    } 
    else 
    {
        return setCommand(_D0, id);
    }
}

std::string mdc2460::cmdChangeBaudrate(unsigned long baudrate)
{
    int baudrate_idx;
    switch (baudrate)
    {
    case 115200:
        baudrate_idx = 0;
        break;
    case 57600:
        baudrate_idx = 1;
        break;
    case 38400:
        baudrate_idx = 2;
        break;
    case 19200:
        baudrate_idx = 3;
        break;
    case 9600:
        baudrate_idx = 4;
        break;
    case 230400:
        baudrate_idx = 10;
        break;
    default:
        throw std::invalid_argument("Error: cannot find the desired Baudrate in the availiable baudrate list!\n");
    }
    return setConfig(_RSBR, baudrate_idx);
}

double mdc2460::mapRange(double val1, double max1, double max2)
{
    double val2 = 0.0;
    if(max1 != 0.0)
        val2 = val1 * max2 / max1;
    return val2;
}

void mdc2460::sleep_ms(unsigned long ms)
{
    usleep(ms*1000);
}
