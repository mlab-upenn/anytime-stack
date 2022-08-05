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

// {18, 19, 21, 22, {}, "MCLK05", "SOC_GPIO42", {13, 15, 18}};

void print_info()
{
    const auto line = "==========================";
    std::cout << line << std::endl;
    std::cout << "[Library Version] " << GPIO::VERSION << std::endl;
    std::cout << "[Model] " << GPIO::model << std::endl;
    std::cout << GPIO::JETSON_INFO;
    std::cout << line << std::endl;
}

class GPIORead : public rclcpp::Node
{
    string channel;
    int state;

	public:
	
	GPIORead() : Node("gpio_read")
	{
        channel = "SOC_GPIO42";
        state = 0;
        GPIO::setwarnings(false);
        GPIO::setmode(GPIO::TEGRA_SOC);
        GPIO::setup(channel, GPIO::IN);
        // GPIO::add_event_detect(channel, GPIO::RISING);
		publisher_ = this->create_publisher<std_msgs::msg::Int32>("/output_read", 1);
		timer_ = this->create_wall_timer(250ms, [this]{ timer_callback(); });
	}

    private:
	
	void timer_callback()
	{
        // state ^= (int) GPIO::event_detected(channel);
		auto out_msg = std_msgs::msg::Int32();
		out_msg.data = GPIO::input(channel);
        // cout << (int) GPIO::event_detected(channel);
		publisher_->publish(out_msg);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  GPIO::cleanup();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPIORead>());
  rclcpp::shutdown();
  return 0;
}