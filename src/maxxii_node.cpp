#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_maxxii/driver_interface.h"
#include "ros2_maxxii/encoder.h"
#include "ros2_maxxii/motor.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#define MILLI 1000.0


class MaxxiiRobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_fbk;
    rclcpp::TimerBase::SharedPtr timer_sys;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sys_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_pub;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub;

    std::shared_ptr<RoboteqDriver> Driver;
    std::shared_ptr<Encoder> EncoderLeft;
    std::shared_ptr<Encoder> EncoderRight;
    std::shared_ptr<Motor> MotorLeft;
    std::shared_ptr<Motor> MotorRight;
    bool enable_encoder_topic;
    bool enable_system_topic;
    bool enable_io_topic;

    double getTimeSec()
    {
        return this->get_clock()->now().seconds();
    }

public:
    MaxxiiRobotNode() : rclcpp::Node("maxxii")
    {
        double ppr, max_rpm, max_tau, max_amp;
        std::string port;
        int rate_feedback, rate_system;
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("motor_max_rpm", 1500.0);
        this->declare_parameter("motor_max_torque_Nm", 1.0);
        this->declare_parameter("motor_max_current_amp", 60.0);
        this->declare_parameter("enable_system_topic", true);
        this->declare_parameter("enable_encoder_topic", true);
        this->declare_parameter("enable_io_topic", false);
        this->declare_parameter("pub_rate_system_Hz", 1);
        this->declare_parameter("pub_rate_feedback_Hz", 20);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("motor_max_current_amp", max_amp);
        this->get_parameter("motor_max_torque_Nm", max_tau);
        this->get_parameter("enable_system_topic", enable_system_topic);
        this->get_parameter("enable_encoder_topic", enable_encoder_topic);
        this->get_parameter("enable_io_topic", enable_io_topic);
        this->get_parameter("pub_rate_system_Hz", rate_system);
        this->get_parameter("pub_rate_feedback_Hz", rate_feedback);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            Driver.reset(new RoboteqDriver(max_rpm, max_amp, max_tau, port));
            EncoderLeft.reset(new Encoder(LEFT_ID, ppr, Driver));
            EncoderRight.reset(new Encoder(RIGHT_ID, ppr, Driver));
            MotorLeft.reset(new Motor(LEFT_ID, Driver));
            MotorRight.reset(new Motor(RIGHT_ID, Driver));
        }
        catch(int code){
            printErrorCode(code);
        }

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        sys_pub = this->create_publisher<std_msgs::msg::String>("doretta/sys", 1);
        enc_pub = this->create_publisher<sensor_msgs::msg::JointState>("doretta/wheels", 1);
        io_pub = this->create_publisher<std_msgs::msg::String>("doretta/io", 1);

        cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/command", qos, 
            std::bind(&MaxxiiRobotNode::cmdCallback, this, std::placeholders::_1)
            );

        timer_fbk = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_feedback)), 
            std::bind(&MaxxiiRobotNode::feedbackCallback, this)
            );
        timer_sys = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_system)), 
            std::bind(&MaxxiiRobotNode::systemCallback, this)
            );
        Driver->printFirmware();
    }
    ~MaxxiiRobotNode(){}

    void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        try{
            if(MotorLeft->isVelocityMode() && MotorRight->isVelocityMode()){
                setMotorSpeeds(joint_state_msg);
            } else if(MotorLeft->isTorqueMode() && MotorRight->isTorqueMode()){
                setMotorTorques(joint_state_msg);
            } else{
                throw DIFFERENT_MOTOR_MODES;
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void setMotorSpeeds(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (2 != joint_state_msg->velocity.size())
            throw WRONG_SIZE_JOINTSTATE_CMD;
        double motor_left_speed = joint_state_msg->velocity[0];
        double motor_right_speed = joint_state_msg->velocity[1];
        MotorLeft->moveRADPS(motor_left_speed);
        MotorRight->moveRADPS(motor_right_speed);
    }
    void setMotorTorques(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (2 != joint_state_msg->effort.size())
            throw WRONG_SIZE_JOINTSTATE_CMD;
        double motor_left_tau = joint_state_msg->effort[0];
        double motor_right_tau = joint_state_msg->effort[1];
        MotorLeft->applyTorque(motor_left_tau);
        MotorRight->applyTorque(motor_right_tau);
    }

    void systemCallback() 
    {
        try{
            if(enable_system_topic)
            {
                sendSystemStateMessage();
            }
            if(enable_io_topic)
            {
                sendIOMessage();
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void feedbackCallback() 
    {
        try{
            if(enable_encoder_topic)
            {
                sendEncoderMessage();
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void sendSystemStateMessage()
    {
        double temp_left = MotorLeft->getTemperature();
        double temp_right = MotorRight->getTemperature();
        double volt_left = MotorLeft->getVoltage();
        double volt_right = MotorRight->getVoltage();
        double ampere_left = MotorLeft->getCurrent();
        double ampere_right = MotorRight->getCurrent();

        std::stringstream ss;
        ss  <<"t="<< getTimeSec()
            <<",Temp=["<<temp_left<<","<<temp_right
            <<"],volt=["<<volt_left<<","<<volt_right
            <<"],amp=["<<ampere_left<<","<<ampere_right<<"]";

        std_msgs::msg::String msg_sys;
        msg_sys.data = ss.str();
        sys_pub->publish(msg_sys);
    }

    void sendEncoderMessage()
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {"left", "right"};
        msg.position = std::vector<double>{
            EncoderLeft->getRadiants(),
            EncoderRight->getRadiants()};
        msg.velocity = std::vector<double>{
            EncoderLeft->getSpeedRPM(),
            EncoderRight->getSpeedRPM()};
        msg.effort = std::vector<double>{
            MotorLeft->getCurrent(),
            MotorRight->getCurrent()};

        enc_pub->publish(msg);
    }

    void sendIOMessage()
    {
        std::string aio = "";
        Driver->getInOutA(&aio);
        std::string dio = "";
         Driver->getInOutD(&dio);

        std::stringstream ss;
        ss  <<"t="<< getTimeSec() 
            <<",aio=["<<aio
            <<"],dio=["<<dio<<"]";

        std_msgs::msg::String msg_io;
        msg_io.data = ss.str();
        io_pub->publish(msg_io);
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MaxxiiRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
