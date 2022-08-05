#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <JetsonGPIO.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;

class GPIOWrite : public rclcpp::Node
{
    string channel;
    int state;

	public:
	
	GPIOWrite() : Node("gpio_write")
	{
        channel = "SOC_GPIO42";
        state = 0;
        GPIO::setwarnings(false);
        GPIO::setmode(GPIO::TEGRA_SOC);
        GPIO::setup("SOC_GPIO42", GPIO::OUT);
		publisher_ = this->create_publisher<std_msgs::msg::Int32>("/output_write", 1);
		timer_ = this->create_wall_timer(1000ms, [this]{ timer_callback(); });
	}

    private:
	
    void timer_callback()
	{
		auto out_msg = std_msgs::msg::Int32();
		out_msg.data = state;
        GPIO::output("SOC_GPIO42", state);
		publisher_->publish(out_msg);
        state ^= 1;
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  GPIO::cleanup();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPIOWrite>());
  rclcpp::shutdown();
  return 0;
}