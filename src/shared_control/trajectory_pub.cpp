#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_data_computed.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace trajectory_msgs::msg;
using namespace std;

class ShotTypeExecuter : public rclcpp::Node {
public:
	ShotTypeExecuter() : Node("shot_type_executer") {
		//publisher
		publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/firefly/command/trajectory", 10);
		
		//subscriber
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
					std::cout << "timestamp: " << msg->timestamp << std::endl;
				});
		vehicle_position_sub_ =
			this -> create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out", 10,
				[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
					vehicle_x_  = msg -> x;
					vehicle_y_ = msg -> y;
					vehicle_z_ = msg -> z;
					//std::cout <<"x position:" << target_x_ << "\t" << "y position:" << target_y_ << "\t" << "z position:" << target_z_ << std::endl;
					//std::cout <<"-------------------------------"<< std::endl;
				});
		target_position_sub_ = 
				this -> create_subscription<nav_msgs::msg::Odometry>("target/odom", 10,
				[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
					target_x_ = msg -> pose.pose.position.x;
					target_y_ = msg -> pose.pose.position.y;
					target_z_ = msg -> pose.pose.position.z;
					std::cout <<"target x position:" << target_x_ << "\t" << "target y position:" << target_y_ << "\t" << "target z position:" << target_z_ << std::endl;
					std::cout <<"-------------------------------"<< std::endl;
					return;
				});
		/*sensor_combined_sub_ =
				this->create_subscription<px4_msgs::msg::SensorDataComputed>("target/sensor_combined", 10,
				[this](const px4_msgs::msg::SensorDataComputed::UniquePtr msg){
					target_x_ = msg -> target_x;
					target_y_ = msg -> target_y;
					target_z_ = msg -> target_z;
					return;
				});*/
				
		auto timer_callback = [this]() -> void {
			std::cout << "loop" << std::endl;
			//TrajectoryGenerator();
			MultiDOFJointTrajectory msg;
      			msg.header.stamp = this->now();
			//auto cycle_duration = this->now() - start_time;
			MultiDOFJointTrajectoryPoint point_msg{};
			std::cout << "Point_msg initialized" << std::endl;
	
			//point_msg.time_from_start = cycle_duration;
			point_msg.transforms.resize(1);
			point_msg.transforms[0].translation.x = target_y_;
			point_msg.transforms[0].translation.y = target_x_;
			point_msg.transforms[0].translation.z = 5;
			std::cout << "Point_msg received" << std::endl;
			
      			msg.joint_names.clear();
      			msg.points.clear();
      			msg.joint_names.push_back("base_link");
      			msg.points.push_back(point_msg);
			std::cout << "Point_msg pushed back" << std::endl;
	
			//msg.transforms.translation.x = 5;
			//msg.transforms.translation.y = 10;
			//msg.transforms.translation.z = 10;
        
			publisher_ -> publish(msg);
			cout << "published" << endl;
		};
		timer_ = this -> create_wall_timer(10ms, timer_callback);
	}


private:

	rclcpp::Publisher<MultiDOFJointTrajectory>::SharedPtr publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	//rclcpp::Subscription<px4_msgs::msg::SensorDataComputed> sensor_combined_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_position_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_position_sub_;
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Time start_time = this->now();
	float vehicle_x_, vehicle_xv_;
	float vehicle_y_, vehicle_yv_;
	float vehicle_z_, vehicle_zv_;
	float target_x_;
	float target_y_;
	float target_z_;
	int shot_type_;
/*	enum shot_type_enum = {
		MAPMP = 1,
		MPTMP = 2,
		LTS = 3,
		VTS = 4,
		PST = 5,
		RS = 6,
		ORBIT = 7,
		FLYOVER = 8,
		FLYBY = 9,
		CHASS = 10
	};*/
	std::atomic<uint64_t> timestamp_;
	void PublishShotTypeTrajectory();
	void TrajectoryGenerator();
};

/*void  ShotTypeExecuter::PublishShotTypeTrajectory(){
	MultiDOFJointTrajectory msg;
      	msg.header.stamp = rclcpp::Clock().now();
	auto cycle_duration = rclcpp::Clock().now() - start_time;
	
	MultiDOFJointTrajectoryPoint point_msg;
	std::cout << "Point_msg initialized" << std::endl;
	
	point_msg.time_from_start = cycle_duration;
	point_msg.transforms[0].translation.x = 5;
	point_msg.transforms[0].translation.y = 5;
	point_msg.transforms[0].translation.z = 10;
	
      	msg.joint_names.clear();
      	msg.points.clear();
      	msg.joint_names.push_back("base_link");
      	msg.points.push_back(point_msg);
	std::cout << "Point_msg pushed back" << std::endl;
	
	//msg.transforms.translation.x = 5;
	//msg.transforms.translation.y = 10;
	//msg.transforms.translation.z = 10;
        
	publisher_ -> publish(msg);
}*/

void ShotTypeExecuter::TrajectoryGenerator(){
	vehicle_x_ = target_y_ + 5;
	vehicle_y_ = target_x_;
	vehicle_z_ = 5;
}

int main(int argc, char* argv[]) {
	std::cout << "Starting Shot Type Executer node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	std::cout << "node initiated" << std::endl;
	rclcpp::spin(std::make_shared<ShotTypeExecuter>());
	std::cout << "node spined" << std::endl;
	rclcpp::shutdown();
	std::cout << "node shut down" << std::endl;
	return 0;
}
