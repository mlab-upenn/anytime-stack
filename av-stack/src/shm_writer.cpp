#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std;

class SHMWrite : public rclcpp::Node
{
	public:

	key_t key = ftok("shmfile", 65);
	int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
    int counter = 0;
	
	SHMWrite() : Node("SHMWrite")
	{
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/write_data", 1);
        timer_ = this->create_wall_timer(1000ms, [this]{ timer_callback(); });
	}

	private:
	
	void timer_callback()
	{
     int *data = (int*) shmat(shmid, (void*)0, 0);
     *data = counter;
	 auto drive_msg = std_msgs::msg::Int32();
	 drive_msg.data = counter;
	 publisher_->publish(drive_msg);
     counter ++;
	}

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
	
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHMWrite>());
  rclcpp::shutdown();
  return 0;
}
