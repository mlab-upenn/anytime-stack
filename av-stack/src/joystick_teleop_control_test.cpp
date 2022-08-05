#include <chrono>
#include <memory>
#include <vector>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyTeleop : public rclcpp::Node
{
	std::vector<float> axs = std::vector<float>(8, 0.0);
	
	public:
	
	JoyTeleop() : Node("joystick_teleop")
	{
		publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
		subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr msg){ process_joystick(msg); });
		timer_ = this->create_wall_timer(10ms, [this]{ timer_callback(); });
	}
	
	private:
	
	void timer_callback()
	{
		auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
		drive_msg.drive.steering_angle = 0.31*axs.at(0);
		drive_msg.drive.speed = (0.75*(axs.at(2) - axs.at(5)))/2; 
		publisher_->publish(drive_msg);
	}
	
	void process_joystick(const sensor_msgs::msg::Joy::SharedPtr joy_in)
	{
		axs = joy_in.get()->axes;
	}
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
