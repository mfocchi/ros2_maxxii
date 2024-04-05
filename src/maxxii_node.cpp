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
    rclcpp::TimerBase::SharedPtr timer_enc_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

    std::shared_ptr<Encoder> encoder_l_;
    std::shared_ptr<Encoder> encoder_r_;
    std::shared_ptr<Motor> motor_l_;
    std::shared_ptr<Motor> motor_r_;
    std::shared_ptr<serial::Serial> com_handle_;

public:
    MaxxiiRobotNode() : rclcpp::Node("maxxii")
    {
        double ppr, tau_const;
        double max_rpm, max_amp;
        std::string port;
        int rate_enc;
        unsigned long baud;

        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baud", 115200);
        this->declare_parameter("motor_max_rpm", 1500.0);
        this->declare_parameter("motor_max_current_amp", 60.0);
        this->declare_parameter("pub_rate_enc_Hz", 20);
        this->declare_parameter("pulse_per_revolution", 1024.0);
        this->declare_parameter("torque_constant", 0.01);

        this->get_parameter("port", port);
        this->get_parameter("baud", baud);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("motor_max_current_amp", max_amp);
        this->get_parameter("pub_rate_enc_Hz", rate_enc);
        this->get_parameter("pulse_per_revolution", ppr);
        this->get_parameter("torque_constant", tau_const);


        try{
            com_handle_.reset(new serial::Serial(port, baud));
            encoder_l_.reset(new Encoder(ppr));
            encoder_r_.reset(new Encoder(ppr));
            motor_l_.reset(new Motor(VELOCITY, tau_const, max_rpm, max_amp));
            motor_r_.reset(new Motor(VELOCITY, tau_const, max_rpm, max_amp));

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
            
            enc_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("enc", 1);
            cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/cmd", qos, 
                std::bind(&MaxxiiRobotNode::cmdCallback, this, std::placeholders::_1)
                );

            timer_enc_ = this->create_wall_timer(
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

    void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try{
            double value_l_norm = computeControl(msg, motor_l_);
            double value_r_norm = computeControl(msg, motor_r_);

            std::string cmd_l = mdc2460::cmdMotor(LEFT, value_l_norm);
            std::string cmd_r = mdc2460::cmdMotor(RIGHT, value_r_norm);

            com_handle_->write(cmd_l);
            com_handle_->write(cmd_r);
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        } 
    }

    double computeControl(const sensor_msgs::msg::JointState::SharedPtr msg, std::shared_ptr<Motor> motor)
    {
        double value_norm = 0.0;
        if(motor->isVelocityMode())
        {
            setMotorSpeeds(msg);
            double vel_rpm = motor->getDesiredVelocity(RPM);
            double max_rpm = motor->getMaxRPM();
            value_norm = mdc2460::mapRange(vel_rpm, max_rpm, SERIAL_MAX_VAL);
        } 
        else if(motor->isTorqueMode())
        {
            setMotorTorques(msg);
            double current = motor->getDesiredCurrent();
            double max_amp = motor->getMaxAmp();
            value_norm = mdc2460::mapRange(current, max_amp, SERIAL_MAX_VAL);
        } 
        else 
        {
            throw std::logic_error("Error: trying to use velocity/torque mode when it is not enabled.\n");
        }
        return value_norm;
    }

    // assumption: message has velocities in rad/s
    void setMotorSpeeds(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->velocity.size() != 2)
            throw std::invalid_argument("Error: received a joint state message with 'velocity' size different than 2.\n");
        double vel_l_radps = joint_state_msg->velocity[0];
        double vel_r_radps = joint_state_msg->velocity[1];
        motor_l_->setDesiredVelocity(vel_l_radps, RADPS);
        motor_r_->setDesiredVelocity(vel_r_radps, RADPS);
    }

    void setMotorTorques(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->effort.size() != 2)
            throw std::invalid_argument("Error: received a joint state message with 'effort' size different than 2.\n");
        double motor_l_tau = joint_state_msg->effort[0];
        double motor_r_tau = joint_state_msg->effort[1];
        motor_l_->setDesiredTorque(motor_l_tau);
        motor_r_->setDesiredTorque(motor_r_tau);
    }

    void encCallback() 
    {
        std::string cmd = mdc2460::reqEncodersCount();        
        com_handle_->write(cmd);

        std::string response = com_handle_->read(BUFFER_SIZE); 
        if(response.length() == 0)
        {
            return;
        } 
        double pulses_l = mdc2460::extractValueLong(response, LEFT);
        double pulses_r = mdc2460::extractValueLong(response, RIGHT);

        encoder_l_->setPulseCount(pulses_l);
        encoder_r_->setPulseCount(pulses_r);

        sensor_msgs::msg::JointState msg;

        msg.name = {"left", "right"};
        msg.position = std::vector<double>{
            encoder_l_->getRadiants(),
            encoder_r_->getRadiants()};

        enc_pub_->publish(msg);
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MaxxiiRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
