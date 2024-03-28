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
#define SERIAL_MAX_VAL 1000.0
#define BUFFER_SIZE 1024

class MaxxiiRobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_enc;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub;

    std::shared_ptr<Encoder> encoder_left_;
    std::shared_ptr<Encoder> encoder_right_;
    std::shared_ptr<Motor> motor_left_;
    std::shared_ptr<Motor> motor_right_;
    std::shared_ptr<serial::Serial> com_handle_;
    double max_rpm_;
    double max_amp_;

public:
    MaxxiiRobotNode() : rclcpp::Node("maxxii")
    {
        double ppr;
        std::string port;
        int rate_enc;
        unsigned long baud;

        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baud", 9600);
        this->declare_parameter("motor_max_rpm", 1500.0);
        this->declare_parameter("motor_max_current_amp", 60.0);
        this->declare_parameter("pub_rate_enc_Hz", 20);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("baud", baud);
        this->get_parameter("motor_max_rpm", max_rpm_);
        this->get_parameter("motor_max_current_amp", max_amp_);
        this->get_parameter("pub_rate_enc_Hz", rate_enc);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            com_handle_.reset(new serial::Serial(port, baud));
            encoder_left_.reset(new Encoder(ppr));
            encoder_right_.reset(new Encoder(ppr));
            motor_left_.reset(new Motor());
            motor_right_.reset(new Motor());

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
            
            enc_pub = this->create_publisher<sensor_msgs::msg::JointState>("enc", 1);
            cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/cmd", qos, 
                std::bind(&MaxxiiRobotNode::cmdCallback, this, std::placeholders::_1)
                );

            timer_enc = this->create_wall_timer(
                std::chrono::milliseconds(int(MILLI / rate_enc)), 
                std::bind(&MaxxiiRobotNode::encCallback, this)
                );
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }   
    }

    ~MaxxiiRobotNode(){com_handle_->close();}

    void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        try{
            double value_l_norm = 0.0;
            double value_r_norm = 0.0;

            if(motor_left_->isVelocityMode() && motor_right_->isVelocityMode()){
                setMotorSpeeds(joint_state_msg);
                double vel_l_rpm = motor_left_->getDesiredVelocity(RPM);
                double vel_r_rpm = motor_right_->getDesiredVelocity(RPM);

                value_l_norm = mapRange(vel_l_rpm, max_rpm_, SERIAL_MAX_VAL);
                value_r_norm = mapRange(vel_r_rpm, max_rpm_, SERIAL_MAX_VAL);

            } else if(motor_left_->isTorqueMode() && motor_right_->isTorqueMode()){
                setMotorTorques(joint_state_msg);
                double current_l = motor_left_->getDesiredCurrent();
                double current_r = motor_right_->getDesiredCurrent();

                value_l_norm = mapRange(current_l, max_amp_, SERIAL_MAX_VAL);
                value_r_norm = mapRange(current_r, max_amp_, SERIAL_MAX_VAL);

                
            } else {
                throw std::logic_error("Error: trying to use velocity/torque mode when it is not enabled.\n");
            }

            std::string cmd_l = sendMotorCmd(LEFT, value_l_norm);
            std::string cmd_r = sendMotorCmd(RIGHT, value_r_norm);

            com_handle_->write(cmd_l);
            com_handle_->write(cmd_r);
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
            throw std::invalid_argument("Error: received a joint state message with 'velocity' size different than 2.\n");
        double vel_l_radps = joint_state_msg->velocity[0];
        double vel_r_radps = joint_state_msg->velocity[1];
        motor_left_->setDesiredVelocity(vel_l_radps, RADPS);
        motor_right_->setDesiredVelocity(vel_r_radps, RADPS);
    }

    void setMotorTorques(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->effort.size() != 2)
            throw std::invalid_argument("Error: received a joint state message with 'effort' size different than 2.\n");
        double motor_left_tau = joint_state_msg->effort[0];
        double motor_right_tau = joint_state_msg->effort[1];
        motor_left_->setDesiredTorque(motor_left_tau);
        motor_right_->setDesiredTorque(motor_right_tau);
    }

    void encCallback() 
    {
        std::string cmd = readEncodersCount();        
        com_handle_->write(cmd);

        std::string response = com_handle_->read(BUFFER_SIZE);  
        double pulses_l = extractValueLong(response, LEFT);
        double pulses_r = extractValueLong(response, RIGHT);

        encoder_left_->setPulseCount(pulses_l);
        encoder_right_->setPulseCount(pulses_r);

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
