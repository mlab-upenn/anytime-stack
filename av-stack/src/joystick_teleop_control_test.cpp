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
		subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&JoyTeleop::process_joystick, this, _1));
		timer_ = this->create_wall_timer(10ms, std::bind(&JoyTeleop::timer_callback, this));
	}
	
	private:
	
	void timer_callback()
	{
		auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
		drive_msg.drive.steering_angle = 0.50*axs.at(0);
		//drive_msg.drive.speed = (1.570*(axs.at(2) - axs.at(5))/2 + 0.00*std::copysign(0.0, axs.at(2) - axs.at(5))); 
		drive_msg.drive.speed = (2.25*(axs.at(2) - axs.at(5)))/2; 
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
