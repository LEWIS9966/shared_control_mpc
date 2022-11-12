#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_data_computed.hpp>
#include <mav_msgs/msg/joystick.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

#define JOYSTICK_RANGE 5.0
#define MARKER_KEEPLAST 700

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace trajectory_msgs::msg;
using namespace std;
using namespace visualization_msgs::msg;

class ShotTypeExecuter : public rclcpp::Node {
public:
	ShotTypeExecuter() : Node("shot_type_executer") {
		//publisher
		publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/sc/trajectory_setpoint", 10);
		target_marker_publisher_ = this->create_publisher<Marker>("/sc/target_pos",10);
		//subscriber
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
					//std::cout << "timestamp: " << msg->timestamp << std::endl;
				});
		target_position_sub_ = 
				this -> create_subscription<nav_msgs::msg::Odometry>("target/odom", 10,
				[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
					target_x_ = msg -> pose.pose.position.x;
					target_y_ = msg -> pose.pose.position.y;
					target_z_ = msg -> pose.pose.position.z;
					return;
				});
		joystick_input_sub_ = 
				this -> create_subscription<mav_msgs::msg::Joystick>("/sc/joystick_input", 10,
				[this](const mav_msgs::msg::Joystick::SharedPtr msg) {
					if(abs(msg -> lx) > 0.1) joystick_x_ = msg->lx; 
					else joystick_x_ = 0.00000;
					if(abs(msg -> ly) > 0.1) joystick_y_ = -msg->ly;
					else joystick_y_ = 0.00000;
					return;
				});
		vehicle_position_sub_ = 
			this -> create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/vehicle_odometry/out", 10,
				[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
					vehicle_x_ = msg -> x;
					vehicle_y_ = msg -> y;
					return;
				});
				
		auto timer_callback = [this]() -> void {
			RCLCPP_INFO(this->get_logger(),"Timer callback:\n");
			//TrajectoryGenerator();
			calculateYawAngle();
			publishTargetSetpoint();
			publishTargetPosition();
		};
		timer_ = this -> create_wall_timer(50ms, timer_callback);
	}


private:
	rclcpp::Publisher<MultiDOFJointTrajectory>::SharedPtr publisher_;
	rclcpp::Publisher<Marker>::SharedPtr target_marker_publisher_;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_position_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_position_sub_;
	rclcpp::Subscription<mav_msgs::msg::Joystick>::SharedPtr joystick_input_sub_;
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Time start_time = this->now();
	float vehicle_x_, vehicle_xv_;
	float vehicle_y_, vehicle_yv_;
	float vehicle_z_, vehicle_zv_;
	float target_x_;
	float target_y_;
	float target_z_;
	float joystick_x_;
    float joystick_y_;
	float yaw_ref_;
	int target_marker_countdown_;
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
	void publishTargetPosition();
	void publishTargetSetpoint();
	void calculateYawAngle();
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
	//msg.transforms.translationLinearModelPredictiveControShotTypeExecuterl.z = 10;
        
	publisher_ -> publish(msg);
}*/


void ShotTypeExecuter::publishTargetPosition(){
  	visualization_msgs::msg::Marker marker_msg;
  	marker_msg.type = Marker::SPHERE;
	if(target_marker_countdown_ < MARKER_KEEPLAST) marker_msg.id = target_marker_countdown_++;
	else marker_msg.id = target_marker_countdown_ = 0;
  	marker_msg.header.frame_id = "map";
  	marker_msg.header.stamp = this->now();
  	marker_msg.action = Marker::ADD;
  	marker_msg.scale.x = 0.1;
  	marker_msg.scale.y = 0.1;
  	marker_msg.scale.z = 0.1;
  	marker_msg.color.a = 1.0; // Don't forget to set the alpha!
  	marker_msg.color.r = 0.0;
  	marker_msg.color.g = 0.0;
  	marker_msg.color.b = 1.0;
  	marker_msg.lifetime = rclcpp::Duration(100);
  	marker_msg.pose.position.x = target_x_+ joystick_x_ * 5.0;
  	marker_msg.pose.position.y = target_y_ + joystick_y_ * 5.0;
  	marker_msg.pose.position.z = 3.5;
	
  	target_marker_publisher_ -> publish(marker_msg);
}


void ShotTypeExecuter::calculateYawAngle(){
	float dx = vehicle_x_ - target_x_;
	float dy = vehicle_y_ - (-target_y_);
	yaw_ref_ = atan2(dy, dx);
	RCLCPP_INFO(this->get_logger(), "relative yaw: %f", yaw_ref_);
}

void ShotTypeExecuter::TrajectoryGenerator(){
	vehicle_x_ = target_y_;
	vehicle_y_ = target_x_;
	vehicle_z_ = 5;
}

void ShotTypeExecuter::publishTargetSetpoint(){
	MultiDOFJointTrajectory msg;
    msg.header.stamp = this->now();
	msg.header.frame_id = "map";
	//auto cycle_duration = this->now() - start_time;
	MultiDOFJointTrajectoryPoint point_msg{};
	//point_msg.time_from_start = cycle_duration;
	point_msg.transforms.resize(1);
	point_msg.velocities.resize(1);
	point_msg.accelerations.resize(1);
	point_msg.transforms[0].translation.x = target_y_ ;//+ joystick_x_ * 5.0;
	point_msg.transforms[0].translation.y = target_x_ ;//+ joystick_y_ * 5.0;
	// point_msg.transforms[0].translation.x = 1;
	// point_msg.transforms[0].translation.y = 1;
	point_msg.transforms[0].translation.z = -3.5;
	// tf2::Quaternion q_tf;
	// q_tf.setRPY(0,0, yaw_ref_);
	// q_tf.normalize();
	// geometry_msgs::msg::Quaternion q_msg;
	// q_msg = tf2::toMsg(q_tf);
	// point_msg.transforms[0].rotation.w = q_msg.w;
	// point_msg.transforms[0].rotation.x = q_msg.x;
	// point_msg.transforms[0].rotation.y = q_msg.y;
	// point_msg.transforms[0].rotation.z = q_msg.z;
	point_msg.velocities[0].linear.x = 0;
	point_msg.velocities[0].linear.y = 0;
	point_msg.velocities[0].linear.z = 0;
	point_msg.accelerations[0].linear.x = 0;
	point_msg.accelerations[0].linear.y = 0;
	point_msg.accelerations[0].linear.z = 0;

    msg.joint_names.clear();
    msg.points.clear();
    msg.points.push_back(point_msg);
	RCLCPP_INFO(this->get_logger(),"joystick_x:%.6f\tjoystick_y: %.6f\n", joystick_x_, joystick_y_);
	RCLCPP_INFO(this->get_logger(),"position reference: pos_x: %.6f\tpos_y: %.6f\tpos_z: %.6f\n", point_msg.transforms[0].translation.x, point_msg.transforms[0].translation.y, point_msg.transforms[0].translation.z);
	//msg.transforms.translation.x = 5;
	//msg.transforms.translation.y = 10;
	//msg.transforms.translation.z = 10;
	publisher_ -> publish(msg);
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
