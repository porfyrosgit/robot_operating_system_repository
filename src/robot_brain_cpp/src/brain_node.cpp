//C++ node decision logic. subscribes to imu_mock topic ,where the mock_imu.py publishes mock imu sensor data.
// step by step: 
// ✔ Subscribes to /imu_mock
// ✔ Publishes to /posture_command
// ✔ Threshold logic 
// ✔ Node spins

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

class BrainNode : public rclcpp::Node
{
public:
    BrainNode() : Node("robot_brain_node")     //name of the node 
    {
        // Subscriber: Listening to the 'Reflex' (Mock IMU)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(      //is a template parameter specifying the message type this subscriber will receive (IMU sensor data)
            "/imu_mock", // topic name the subscriber will listen to.Any messages published to "/imu_mock" will be received by this subscriber
            10,              //QualityOfService queue size. the subscriber will buffer up to 10 messages if they arrive faster than they can be processed
            std::bind(&BrainNode::imu_callback, this, std::placeholders::_1)
        );

        // Publisher: Sending the 'Intent' (Posture Command)
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/posture_command",           //This line creates the topic /posture_command.
            10
        );
        // Timer: runs every 100 ms
        timer_ = this->create_wall_timer(
             std::chrono::milliseconds(100),
              std::bind(&BrainNode::control_loop, this) 
            );
        

        RCLCPP_INFO(this->get_logger(), "Robot Brain Node Started");
    }

private:
    // Timer callback
    void control_loop() { std_msgs::msg::String msg; msg.data = "IDLE";    // placeholder logic
         cmd_pub_->publish(msg); 
        }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double tilt = msg->orientation.x; 
        std_msgs::msg::String cmd;

        // Logic: Threshold-based decision making
        if (std::abs(tilt) > 0.5) {
            cmd.data = "STABILIZE";
        } else {
            cmd.data = "OK";
        }

        cmd_pub_->publish(cmd);
    }
    //private class members - values not directly changeable from the outside
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}