#include <gtest/gtest.h>
#include "ros2_maxxii/driver_interface.h"

// Test for runtime queries ------------------------
using namespace mdc2460;

TEST(DriverTest, ReqEncodersCount) {
    std::string result = reqEncodersCount();
    EXPECT_EQ(result, "?$04 0\r");
}

TEST(DriverTest, ReqEncodersSpeed) {
    std::string result = reqEncodersSpeed();
    EXPECT_EQ(result, "?$03 0\r");
}

TEST(DriverTest, ReqFeedback) {
    std::string result = reqFeedback();
    EXPECT_EQ(result, "?$13 0\r");
}

TEST(DriverTest, ReqMotorsCurrent) {
    std::string result = reqMotorsCurrent();
    EXPECT_EQ(result, "?$00 0\r");
}

TEST(DriverTest, ReqMotorsVoltage) {
    std::string result = reqMotorsVoltage();
    EXPECT_EQ(result, "?$0D 0\r");
}

TEST(DriverTest, ReqFirmware) {
    std::string result = reqFirmware();
    EXPECT_EQ(result, "?$1E 0\r");
}

TEST(DriverTest, ReqTemperature) {
    std::string result = reqTemperature();
    EXPECT_EQ(result, "?$12 0\r");
}

// Test for runtime commands ------------------------

TEST(DriverTest, CmdMotor) {
    std::string result = cmdMotor(LEFT,0.0);
    EXPECT_EQ(result, "!$00 1 0\r");

    result = cmdMotor(RIGHT,100.0);
    EXPECT_EQ(result, "!$00 2 100\r");
}

TEST(DriverTest, CmdMotorRPM) {
    std::string result = cmdMotorRPM(RIGHT,0.0);
    EXPECT_EQ(result, "!$03 2 0\r");

    result = cmdMotorRPM(LEFT,155.0);
    EXPECT_EQ(result, "!$03 1 155\r");
}

// Test for config changes ---------------------------

TEST(DriverTest, CmdChangeBaudrate) {
    std::string result = cmdChangeBaudrate(9600);
    EXPECT_EQ(result, "^$0A 4\r");

    result = cmdChangeBaudrate(230400);
    EXPECT_EQ(result, "^$0A 10\r");

    EXPECT_ANY_THROW(cmdChangeBaudrate(142));
}
