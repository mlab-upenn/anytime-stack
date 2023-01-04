#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

# define PI           3.14159265358979323846

// Setup the shared memory file and obtain key and shared memory id (change the filepath to desired filepath)
// Will heap allocate 1024 bytes 

key_t key = ftok("/home/f1tenth3/shmfile", 65);
int shmid = shmget(key, 1024, 0666 | IPC_CREAT);


// Stanley controller class
class Stanley : public rclcpp::Node
{
    float vx, t_ttc, m, c, atan_m_, sec_m_, k_gain, ttc_thresh;
	float L, steering_raw, speed;
	std::vector<float> axs, range;
	std::vector<int> btns;
    bool enableController;

	VectorXf r;
	VectorXf r_dot;
	VectorXf cosBeta;

    public:

    Stanley() : Node("stanley_control")
    {

        // Get parameters from yaml config file

        declare_parameter("k_gain", 0.5);

		declare_parameter("speed", 1.0);

        declare_parameter("slope", 0.0);

		declare_parameter("intercept", -0.73);

        declare_parameter("time_to_collision", 0.75);

        // Create joystick, ICP_odom, and LiDAR subscriptions; Create the ackerman drive topic publisher to control the car; Create wall timer of 25ms

		drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

        // Please make use of lambda expressions instead of std::bind

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr msg){ process_joystick(msg);} );

        icp_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/icp/odom", 1, [this](nav_msgs::msg::Odometry::SharedPtr msg){ process_amcl(msg);} ) ;

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ process_laser(msg);} );

        timer_ = this->create_wall_timer(25ms, [this]{ timer_callback(); });

        // Create a vector of the cosine of the LiDAR bearing angles

        cosBeta = VectorXf::LinSpaced(1081, -3*PI/4, 3*PI/4).array().cos();

        // Obtain all parameters from yaml config file

        m = get_parameter("slope").as_double();

        k_gain = get_parameter("k_gain").as_double();

        k = get_parameter("index").as_int();

        c = get_parameter("intercept").as_double();

        speed = get_parameter("speed").as_double();

        ttc_thresh = get_parameter("time_to_collision").as_double();

        atan_m_ = atan(m);

        sec_m_ = sqrt(1 + m*m);

        speed = 1.0;

        vx = speed;

		steering_raw = 0.0;

        enableController = false;
    }

    private:

    // Joystick reader to teleop start and stop controller

	void process_joystick(const sensor_msgs::msg::Joy::SharedPtr joy_in)
	{
		axs = joy_in.get()->axes;
		btns = joy_in.get()->buttons;

		if (btns[0] == 1)
		{
			enableController = true;
		}
		if (btns[1] == 1)
		{
			enableController = false;
		}
	}

    // LiDAR reader to estimate time to collision

	void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr laser_in)
	{
		range = laser_in.get()->ranges;

		int n = range.size();

        // Get pointer to first element of array

		float* range_ptr = &range[0];

        // Map raw array to Eigen array

		Map<ArrayXf> r(range_ptr, n);

        // Choose a small sliver of the LiDAR beams to compute time to collision

		ArrayXf t_all = r.segment(540-20, 40) / (vx*cosBeta.segment(540-20, 40).array()).cwiseMax(0.00f);

        // Pick the smallest time to collision

		t_ttc = t_all.minCoeff();

        // If estimated time to collision is less than the chosen threshold then e-stop the car

		if(t_ttc <= ttc_thresh)
		{
			enableController = false;
		}
	}

    void process_icp(const nav_msgs::msg::Odometry::SharedPtr icp_odom)
    {
        vx = icp_odom->twist.twist.linear.x;

        // Convert quaterion to heading angle

        float angle = 2*atan2(icp_odom->pose.pose.orientation.z, icp_odom->pose.pose.orientation.w);

        // Get current pose from ICP

        float x = icp_odom->pose.pose.position.x;
        float y = icp_odom->pose.pose.position.y;

        // Calculate crosstracking error between vehicle's current pose and desired trajectory

        float e = (m*x - y + c) / sec_m_;

        // Calculate desired steering angle

        steering_raw = atan_m_ - angle + atan(k*e/vx);
    }

	void timer_callback()
	{

        // Check to see if ICP is done compute the pose estimate via the shared memory handshaking signal

        int *interrupt = (int*) shmat(shmid, (void*) 0, 0);

        if (*(interrupt + 1) != 0)
        {
            *(interrupt + 1) = 0;
            break;
        }

        else
        {
            // Send the max number of iterations to ICP before a collision is inevitable 
            *interrupt = (int)(t_ttc * 40);
        }

        // Publish speed and steering commands to vehicle's motor controller

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;

		drive_msg.drive.steering_angle = steering_raw*int(enableController);
		drive_msg.drive.speed = speed*int(enableController);

		drive_pub_->publish(drive_msg);
	}

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stanley>());
  rclcpp::shutdown();
  return 0;
}