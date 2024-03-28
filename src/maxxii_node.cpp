#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_maxxii/driver_interface.h"
#include "ros2_maxxii/encoder.h"
#include "ros2_maxxii/motor.h"
#include "serial/serial.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#define MILLI 1000.0


class MaxxiiRobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_fbk;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub;

    std::shared_ptr<Encoder> encoder_left_;
    std::shared_ptr<Encoder> encoder_right_;
    std::shared_ptr<Motor> motor_left_;
    std::shared_ptr<Motor> motor_right_;
    std::shared_ptr<serial::Serial> com_handle_;

    double getTimeSec()
    {
        return this->get_clock()->now().seconds();
    }

public:
    MaxxiiRobotNode() : rclcpp::Node("maxxii")
    {
        double ppr, max_rpm, max_tau, max_amp;
        std::string port;
        int rate_enc;
        unsigned long baud;

        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baud", 9600);
        this->declare_parameter("motor_max_rpm", 1500.0);
        this->declare_parameter("motor_max_torque_Nm", 1.0);
        this->declare_parameter("motor_max_current_amp", 60.0);
        this->declare_parameter("pub_rate_enc_Hz", 20);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("baud", baud);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("motor_max_current_amp", max_amp);
        this->get_parameter("motor_max_torque_Nm", max_tau);
        this->get_parameter("enable_system_topic", enable_system_topic);
        this->get_parameter("enable_io_topic", enable_io_topic);
        this->get_parameter("pub_rate_enc_Hz", rate_enc);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            com_handle_.reset(new serial::Serial(port, baud));
            encoder_left_.reset(new Encoder(LEFT_ID, ppr, Driver));
            encoder_right_.reset(new Encoder(RIGHT_ID, ppr, Driver));
            motor_left_.reset(new Motor(LEFT_ID, Driver));
            motor_right_.reset(new Motor(RIGHT_ID, Driver));

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
            
            sys_pub = this->create_publisher<std_msgs::msg::String>("sys", 1);
            enc_pub = this->create_publisher<sensor_msgs::msg::JointState>("enc", 1);
            io_pub = this->create_publisher<std_msgs::msg::String>("io", 1);

            cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/cmd", qos, 
                std::bind(&MaxxiiRobotNode::cmdCallback, this, std::placeholders::_1)
                );

            timer_fbk = this->create_wall_timer(
                std::chrono::milliseconds(int(MILLI / rate_feedback)), 
                std::bind(&MaxxiiRobotNode::encCallback, this)
                );
            timer_sys = this->create_wall_timer(
                std::chrono::milliseconds(int(MILLI / rate_system)), 
                std::bind(&MaxxiiRobotNode::systemCallback, this)
                );
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }   
    }

    ~MaxxiiRobotNode(){com_handle_.close();}

    void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        try{
            if(motor_left_->isVelocityMode() && motor_right_->isVelocityMode()){
                setMotorSpeeds(joint_state_msg);
                double vel_l_rpm = motor_left_->getDesiredVelocity(RPM);
                double vel_r_rpm = motor_right_->getDesiredVelocity(RPM);

            } else if(motor_left_->isTorqueMode() && motor_right_->isTorqueMode()){
                setMotorTorques(joint_state_msg);
                double tau_l = motor_left_->getDesiredTorque();
                double tau_r = motor_right_->getDesiredTorque();

            } else {
                throw logic_error("Error: trying to use velocity/torque mode when it is not enabled.\n");
            }
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        } 
    }

    // assumption: message has velocities in rad/s
    void setMotorSpeeds(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->velocity.size() != 2)
            throw invalid_argument("Error: received a joint state message with 'velocity' size different than 2.\n");
        double vel_l_radps = joint_state_msg->velocity[0];
        double vel_r_radps = joint_state_msg->velocity[1];
        motor_left_->setDesiredVelocity(vel_l_radps, RADPS);
        motor_right_->setDesiredVelocity(vel_r_radps, RADPS);
    }

    void setMotorTorques(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->effort.size() != 2)
            throw invalid_argument("Error: received a joint state message with 'effort' size different than 2.\n");
        double motor_left_tau = joint_state_msg->effort[0];
        double motor_right_tau = joint_state_msg->effort[1];
        motor_left_->setDesiredTorque(motor_left_tau);
        motor_right_->setDesiredTorque(motor_right_tau);
    }

    void encCallback() 
    {
        std::string cmd = readEncodersCount();        
        com_handle_.write(cmd);
        // TODO: check expected end of response char or max length
        // std::string response = com_handle_.read();  
        // std::string response = com_handle_.readline();  
        double pulses_l = extractValueLong(response, LEFT);
        double pulses_r = extractValueLong(response, RIGHT);

        encoder_left_->()

        sensor_msgs::msg::JointState msg;

        msg.name = {"left", "right"};
        msg.position = std::vector<double>{
            encoder_left_->getRadiants(),
            encoder_right_->getRadiants()};

        enc_pub->publish(msg);
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MaxxiiRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
