#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_data_computed.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
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

//defines the shot type(0: lateral tracking shot, 1: ORBOT)
#define SHOT_TYPE 1

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
		publisher_ = 
			this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/sc/trajectory_setpoint", 10);
		trajectory_ref_publisher_ = 
			this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/sc/trajectory_setpoint_combined", 10);
		target_marker_publisher_ = 
			this->create_publisher<Marker>("/sc/target_pos",10);
		offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in",10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in",10);

		//subscriber
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		target_position_sub_ = 
				this -> create_subscription<nav_msgs::msg::Odometry>("/target/odom", 10,
				[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
					target_x_ = msg -> pose.pose.position.x;
					target_y_ = msg -> pose.pose.position.y;
					target_z_ = msg -> pose.pose.position.z;
					return;
				});

		joystick_input_sub_ = 
				this -> create_subscription<mav_msgs::msg::Joystick>("/sc/joystick_input", 10,
				[this](const mav_msgs::msg::Joystick::SharedPtr msg) {
					if(abs(msg -> lx) > 0.7) joystick_x_ = msg->lx; 
					else joystick_x_ = 0.00000;
					if(abs(msg -> ly) > 0.7) joystick_y_ = -msg->ly;
					else joystick_y_ = 0.00000;
					return;
				});

		vehicle_position_sub_ = 
			this -> create_subscription<nav_msgs::msg::Odometry>("/vehicle/odom", 10,
				[this](const nav_msgs::msg::Odometry::SharedPtr msg) {
					vehicle_x_ = msg->pose.pose.position.x;
					vehicle_y_ = msg->pose.pose.position.y;

					euler_angles_[2] = yaw_ref_;
					return;
				});

		trajectory_ref_sub_ = 
			this -> create_subscription<MultiDOFJointTrajectory>("/sc/trajectory_setpoint", 10,
				[this](const MultiDOFJointTrajectory::SharedPtr msg) {
					final_x_ = msg->points[0].transforms[0].translation.x;
					final_y_ = msg->points[0].transforms[0].translation.y;
					return;
				});
		
		loop_ = 5;
		auto timer_callback = [this]() -> void {
			//initial point (5,-5,-2.5)before 10 seconds
			if(counter_ < 100){
				setpoint_x_ = 5;
				setpoint_y_ = -5;
				setpoint_z_ = -2.5;
				counter_++;
			}
			else TrajectoryGenerator();
			calculateYawAngle();
			publishTargetSetpoint();
			#ifdef PID
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			#endif
		};
		timer_ = this -> create_wall_timer(100ms, timer_callback);
	}


private:
	rclcpp::Publisher<MultiDOFJointTrajectory>::SharedPtr publisher_;
	rclcpp::Publisher<MultiDOFJointTrajectory>::SharedPtr trajectory_ref_publisher_;
	rclcpp::Publisher<Marker>::SharedPtr target_marker_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_position_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_position_sub_;
	rclcpp::Subscription<mav_msgs::msg::Joystick>::SharedPtr joystick_input_sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<MultiDOFJointTrajectory>::SharedPtr trajectory_ref_sub_;
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Time start_time = this -> now();
	std::atomic<uint64_t> timestamp_;

	static double frequency_;
	float q_[4];
	float euler_angles_[3];
	float vehicle_x_, vehicle_xv_;
	float vehicle_y_, vehicle_yv_;
	float vehicle_z_, vehicle_zv_;
	float target_x_;
	float target_y_;
	float target_z_;
	float setpoint_x_, setpoint_y_, setpoint_z_;
	float joystick_x_;
    float joystick_y_;
	float final_x_, final_y_; //simple added trajectory for LQR and PID
	float yaw_ref_; //yaw angle towards target
	int target_marker_countdown_;
	int shot_type_;
	int counter_;
	float loop_;
	float r_;


	void PublishShotTypeTrajectory();
	void TrajectoryGenerator();
	void publishTargetPosition();
	void publishTargetSetpoint();
	void calculateYawAngle();
	void publish_trajectory_setpoint() const;
	void publish_offboard_control_mode() const;
	void inline getEulerAnglesFromQuaternion(float q[4], float euler_angles[3])
    {
        assert(euler_angles != NULL);
        euler_angles[0] = std::atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])),
        euler_angles[1] = std::asin(2.0 * (q[0] * q[2] - q[3] * q[1])),
        euler_angles[2] = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }
};

/**
 * @brief publish Target position marker for visualization 
 * 
 */
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
  	marker_msg.color.a = 1.0; 
  	marker_msg.color.r = 0.0;
  	marker_msg.color.g = 0.0;
  	marker_msg.color.b = 1.0;
  	marker_msg.lifetime = rclcpp::Duration(100);
  	marker_msg.pose.position.x = final_x_;
  	marker_msg.pose.position.y = final_y_;
  	marker_msg.pose.position.z = 2.5;
	
  	target_marker_publisher_ -> publish(marker_msg);
}

/**
 * @brief calculate yaw angle between current pose and target pose for marker orientation
 * 
 */
void ShotTypeExecuter::calculateYawAngle(){
	float dx = target_x_ - vehicle_x_;
	float dy = target_y_ - vehicle_y_;
	yaw_ref_ = atan2(dx, dy);
	// RCLCPP_INFO(this->get_logger(), "target_x: %f        target_y: %f", target_x_, target_y_);
	// RCLCPP_INFO(this->get_logger(), "vehicle_x: %f       vehicle_y: %f", vehicle_x_, vehicle_y_);
	// RCLCPP_INFO(this->get_logger(), "relative yaw: %f", yaw_ref_);
	// printf("----------------------------------------------------\n");
}

/**
 * @brief main function for trajectory generation given different shot type
 * 
 */
void ShotTypeExecuter::TrajectoryGenerator(){
	if(SHOT_TYPE ==0){
		setpoint_x_ = 5 + target_x_;
		setpoint_y_ = target_y_;
		setpoint_z_ = -2.5;
	}
	else if(SHOT_TYPE == 1){
		if(loop_ == 361) loop_ = 0;
		r_ = 5;
		setpoint_x_ = target_y_ + r_ * sin(loop_ / 360 * 2 * M_PI);
		setpoint_y_ = target_x_ + r_ * cos(loop_ / 360 * 2 * M_PI);
		setpoint_z_ = -2.5;
		loop_ += 0.5;
	}
}

/**
 * @brief publish final target setpoint
 * 
 */
void ShotTypeExecuter::publishTargetSetpoint(){
	MultiDOFJointTrajectory msg;
    msg.header.stamp = this -> now();
	msg.header.frame_id = "map";
	MultiDOFJointTrajectoryPoint point_msg{};
	point_msg.transforms.resize(1);
	point_msg.velocities.resize(1);
	point_msg.accelerations.resize(1);

	point_msg.transforms[0].translation.x = setpoint_x_;
	point_msg.transforms[0].translation.y = setpoint_y_;
	point_msg.transforms[0].translation.z = setpoint_z_;

	tf2::Quaternion q_tf;
	q_tf.setRPY(0,0, yaw_ref_);
	q_tf.normalize();
	geometry_msgs::msg::Quaternion q_msg;
	q_msg = tf2::toMsg(q_tf);
	point_msg.transforms[0].rotation.w = q_msg.w;
	point_msg.transforms[0].rotation.x = q_msg.x;
	point_msg.transforms[0].rotation.y = q_msg.y;
	point_msg.transforms[0].rotation.z = q_msg.z;
	point_msg.velocities[0].linear.x = 0;
	point_msg.velocities[0].linear.y = 0;
	point_msg.velocities[0].linear.z = 0;
	point_msg.accelerations[0].linear.x = 0;
	point_msg.accelerations[0].linear.y = 0;
	point_msg.accelerations[0].linear.z = 0;

    msg.joint_names.clear();
    msg.points.clear();
    msg.points.push_back(point_msg);
	// RCLCPP_INFO(this->get_logger(),"joystick_x:%.6f\tjoystick_y: %.6f\n", joystick_x_, joystick_y_);
	RCLCPP_INFO(this->get_logger(),"position reference: pos_x: %.6f\tpos_y: %.6f\tpos_z: %.6f\n", point_msg.transforms[0].translation.x, point_msg.transforms[0].translation.y, point_msg.transforms[0].translation.z);
	//运镜轨迹
	publisher_ -> publish(msg);

	msg.header.stamp = this -> now();
	msg.points.clear();
	//transform to body frame
	point_msg.transforms[0].translation.x = final_x_= setpoint_x_ + (joystick_y_ * 2) * cos(yaw_ref_ - 1.5706) - (joystick_x_ * 2) * sin(yaw_ref_ - 1.5706);
	point_msg.transforms[0].translation.y = final_y_ = setpoint_y_ + (joystick_y_ * 2) * sin(yaw_ref_ - 1.5706) + (joystick_x_ * 2) * cos(yaw_ref_ - 1.5706);
	point_msg.transforms[0].translation.z = setpoint_z_;

    msg.points.push_back(point_msg);
	// RCLCPP_INFO(this->get_logger(),"joystick_x:%.6f\tjoystick_y: %.6f\n", joystick_x_, joystick_y_);
	// RCLCPP_INFO(this->get_logger(),"position reference: pos_x: %.6f\tpos_y: %.6f\tpos_z: %.6f\n", point_msg.transforms[0].translation.x, point_msg.transforms[0].translation.y, point_msg.transforms[0].translation.z);
	trajectory_ref_publisher_ -> publish(msg);
}

/**
 * @brief publish trajectory setpoint directly to PIXHAWK for PID control
 * 
 */
void ShotTypeExecuter::publish_trajectory_setpoint() const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = final_x_ - 1;
	msg.y = final_y_ - 1;
	msg.z = -2.5;
	msg.yaw = yaw_ref_;

	trajectory_setpoint_publisher_->publish(msg);
}

void ShotTypeExecuter::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
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
