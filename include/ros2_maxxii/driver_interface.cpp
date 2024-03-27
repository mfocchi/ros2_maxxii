#include "driver_interface.h"


std::string getCmdVersion()
{
    return issueCommand("?", "FID", 50);
}

std::string mixedModeMotorMove(float throttle, float steering) 
{

    std::stringstream ss;
    ss << "!M" << throttle << " " << steering << "\r";
    return ss.str();
}

std::string issueCommand(std::string commandType, std::string command, std::string args, int waitms, bool isplusminus) {
    std::stringstream ss;
    if (args == "")
        ss << commandType << command << "\r";
    else
        ss << commandType << command << " " << args << "\r";
    return ss.str();
}

std::string issueCommand(std::string commandType, std::string command, int waitms, bool isplusminus) {
    return issueCommand(commandType, command, "", waitms, isplusminus);
}

std::string extractValueString(std::string msg, std::string command) 
{
    std::string::size_type pos = msg.rfind(command + "=");
    if (pos == std::string::npos)
    {
        std::stringstream ss;
        ss << "Could not find character '=' in message '" << msg << std::endl;
        throw std::runtime_error(ss.str());
    }

    pos += command.length() + 1; // +1 is because of the '='

    std::string::size_type carriage = msg.find("\r", pos);
    if (carriage == std::string::npos)
    {
        std::stringstream ss;
        ss << "Could not find character 'Carriage Return' in message '" << msg << std::endl;
        throw std::runtime_error(ss.str());
    }
    return msg.substr(pos, carriage - pos);
}

std::string setConfig(int configItem, int index, int value) 
{
    std::string response;
    char command[10];
    char args[50];

    if (isItemOutRange(configItem))
    {
        std::stringstream ss;
        ss << "Cannot set configuration, item out of reange [0,255] = " << configItem << std::endl;
        throw std::runtime_error(ss.str());
    }

    sprintf(command, "$%02X", configItem);
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

    return issueCommand("^", command, args, 10, true);
}

bool isItemOutRange(int item)
{
    return item < 0 || item > 255;
}

std::string setConfig(int configItem, int value) 
{
    return setConfig(configItem, MISSING_VALUE, value);
}

std::string setCommand(int commandItem, int index, int value) {
    char command[10];
    char args[50];

    if (isItemOutRange(commandItem))
    {
        std::stringstream ss;
        ss << "Cannot set command, item out of reange [0,255] = " << commandItem << std::endl;
        throw std::runtime_error(ss.str());
    }
    sprintf(command, "$%02X", commandItem);
    sprintf(args, "%i %i", index, value);

    // DEBUG
    std::stringstream sdebug;
    sdebug << args;

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
    return issueCommand("!", command, args, 10, true);
}

std::string setCommand(int commandItem, int value) 
{
    return setCommand(commandItem, MISSING_VALUE, value);
}

std::string setCommand(int commandItem) 
{
    return setCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

std::string getConfig(int configItem, int index)
{
    char command[10];
    char args[50];

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

    sprintf(command, "$%02X", configItem);
    sprintf(args, "%i", index);

    return issueCommand("~", command, args, 10);
}

std::string getConfig(int configItem) 
{
    return getConfig(configItem, 0);
}

std::string getValue(int operatingItem, int index) 
{
    char command[10];
    char args[50];

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

    sprintf(command, "$%02X", operatingItem);
    sprintf(args, "%i", index);

    return issueCommand("?", command, args, 10);
}

double extractMeasurement( std::string& reading_str, Position position) 
{ 
    size_t colonPos = reading_str.find(':');
    std::string reading;
    if (position == LEFT) 
    {
        reading = reading_str.substr(0, colonPos);
    }
    else if (position == RIGHT) 
    {
        reading = reading_str.substr(colonPos + 1);
    }
    return std::stoi(reading);
}

std::string readEncodersCount()
{
    return getValue(_C, 0);
}

std::string readEncodersSpeed()
{
    return getValue(_S, 0);
}

std::string readFeedback()
{
    return getValue(_F, 0);
}

std::string readMotorsCurrent()
{
    return getValue(_A, 0);
}

std::string readMotorsVoltage()
{
    return getValue(_V, 0);
}

std::string readFirmware()
{
    return getValue(_FID, 0);
}

std::string readTemperature()
{
    return getValue(_T, 0);
}

std::string resetEncoder(int id)
{
    return setCommand(_C, id, 0);
}

std::string sendMotorCmd(int id, double cmd_value)
{
    return setCommand(_GO, id, cmd_value);
}

std::string commandInputOutput(bool set, Position p) 
{
    // id can be 1 or 2

    // 1  Setting D1 OUT
    // 0  Resetting D1 OUT 
    int id = 1;
    if(p == LEFT)
        id = 1;
    else if(p == RIGHT)
        id = 2;
    if (set) {
        return setCommand(_D1, id);
    } else {
        return setCommand(_D0, id);
    }
}

double mapRange(double val1, double max1, double max2)
{
    double val2 = 0.0;
    if(max1 != 0.0)
        val2 = val1 * max2 / max1;
    return val2;
}

double saturate(double val, double max_val)
{
    if(val > max_val)
        return max_val;
    else if(val < -max_val)
        return -max_val;
    else
        return val;
}

std::string replaceString(std::string source, std::string find, std::string replacement) {
    std::string::size_type pos = 0;
    while ((pos = source.find(find, pos)) != std::string::npos) {
        source.replace(pos, find.size(), replacement);
        pos++;
    }
    return source;
}
