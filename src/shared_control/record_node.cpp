#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mav_msgs/msg/joystick.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <time.h>
#include <string>
#include <math.h>

using namespace std;
using namespace trajectory_msgs::msg;
using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class Record : public rclcpp::Node{
    public:
    Record(): Node("Record_Node"){
        count_ = dist_ = dist_sum_ = 0;
        time_t now = time(0);
        tic_timestart_ = clock();
        string order_path = "//home//lewis//px4_ros_com_ros2//records//";
        string seq1 = order_path + "ref_pos"+".csv";
        string seq2 = order_path + "odom_pos"+".csv";
        string seq3 = order_path + "statistic"+".csv";
        string seq4 = order_path + "attitude_loop" + ".csv";
        output_ref_.open(seq1, ios::out);
        output_odom_.open(seq2, ios::out);
        statistic_.open(seq3, ios::out);
        attitude_loop_.open(seq4,ios::out);
        vehicle_position_sub_ = 
            this->create_subscription<VehicleOdometry>("fmu/vehicle_odometry/out", 10, [this](const VehicleOdometry::SharedPtr msg){
                RCLCPP_INFO(this->get_logger(), "odometry position: x: %.4f   y: %.4f   z: %.4f", msg -> x, msg -> y, msg -> z);
                clock_t tic_timestop = clock();
                odom_x_ = msg -> x;
                odom_y_ = msg -> y;
                odom_z_ = msg -> z;
                roll_rate_ = msg -> rollspeed;
                pitch_rate_ = msg -> pitchspeed;
                output_odom_ << (float)(tic_timestop - tic_timestart_) / CLOCKS_PER_SEC << ',' << msg -> x << "," << msg -> y << "," << msg -> z << "," << roll_rate_ << "," << pitch_rate_ << endl; 
            });
        setpoint_position_sub_ = 
            this->create_subscription<MultiDOFJointTrajectory>("/sc/trajectory_setpoint_combined", 10, [this](const MultiDOFJointTrajectory::SharedPtr msg){
                ref_x_ = msg -> points.front().transforms.front().translation.x;
                ref_y_ = msg -> points.front().transforms.front().translation.y;
                ref_z_ = msg -> points.front().transforms.front().translation.z;
                clock_t tic_timestop = clock();
                RCLCPP_INFO(this->get_logger(), "setpoint position: x: %.4f   y: %.4f   z: %.4f", ref_x_, ref_y_, ref_z_);
                output_ref_ << (float)(tic_timestop - tic_timestart_) / CLOCKS_PER_SEC << ',' << ref_x_ << "," << ref_y_ << "," << ref_z_ << ',' 
                            << joystick_x_ * 5 << ',' << joystick_y_ * 5 << endl; 
            });
        vehicle_attitude_setpoint_sub = 
            this->create_subscription<VehicleAttitudeSetpoint>("/fmu/vehicle_attitude_setpoint/in", 10, [this](const VehicleAttitudeSetpoint::SharedPtr msg){
                quaternion_[0] = msg->q_d[0];
                quaternion_[1] = msg->q_d[1];
                quaternion_[2] = msg->q_d[2];
                quaternion_[3] = msg->q_d[3];
                getEulerAnglesFromQuaternion(quaternion_);
                clock_t tic_timestop = clock();
                attitude_loop_ << (float)(tic_timestop - tic_timestart_) / CLOCKS_PER_SEC << ',' << roll_ref_ << ',' << pitch_ref_ << ',' << roll_rate_ << ',' << pitch_rate_ << endl;
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
        auto timer_callback = [this]() -> void {
            dist_ = std::hypot((odom_x_ - ref_x_), (odom_y_ - ref_y_));
            dist_sum_ += dist_;
            ++count_;
            statistic_ << (float)(clock() - tic_timestart_) / CLOCKS_PER_SEC << ',' << dist_ << endl;
        };
        timer_ = this -> create_wall_timer(20ms, timer_callback);
    };
    ~Record(){
        mae_ = sqrt(dist_sum_ / (float)count_);
        statistic_ << mae_;
        output_ref_.close();
        output_odom_.close();
        statistic_.close();
        attitude_loop_.close();
    };
    void inline getEulerAnglesFromQuaternion(const double q[4])
    {
        roll_ref_ = std::atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
        pitch_ref_ = std::asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
        // euler_angles[0] = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }
    
    private:    
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_position_sub_;
    rclcpp::Subscription<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_sub;
    rclcpp::Subscription<MultiDOFJointTrajectory>::SharedPtr setpoint_position_sub_;
    rclcpp::Subscription<mav_msgs::msg::Joystick>::SharedPtr joystick_input_sub_;
    ofstream output_odom_;
    ofstream output_ref_;
    ofstream statistic_;
    ofstream attitude_loop_;
    clock_t tic_timestart_;
    double ref_x_, ref_y_, ref_z_, odom_x_, odom_y_, odom_z_;
    double dist_, dist_sum_, mae_;
    double joystick_x_, joystick_y_;
    double quaternion_[4];
    double eular_angle_[3];
    double roll_ref_;
    double pitch_ref_;
    double roll_rate_, pitch_rate_;
    long long count_;
    rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Record>());
  rclcpp::shutdown();
  return 0;
}