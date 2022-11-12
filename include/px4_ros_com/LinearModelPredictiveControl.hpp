#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

#include <solver.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <mav_msgs/msg/joystick.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <stdint.h>

#include <px4_ros_com/mpc_queue.hpp>


using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace Eigen;
using namespace visualization_msgs::msg;

namespace shared_control_mpc{
using namespace px4_msgs::msg;

static constexpr int kStateSize = 8;
static constexpr int kInputSize = 3;
static constexpr int kMeasurementSize = 6;
static constexpr int kDisturbanceSize = 3;
static constexpr int kPredictionHorizonSteps = 20;
static constexpr double kGravity = 9.8066;
static constexpr double prediction_sampling_time_ = 0.1;
static constexpr double sampling_time_ = 0.0083;//与odometry频率相同：120Hz
static constexpr double mass_ = 1.52;
static constexpr double roll_limit_ = M_PI / 8.0;
static constexpr double pitch_limit_ = M_PI / 8.0;
static constexpr double yaw_rate_limit_ = M_PI_2;
static constexpr double thrust_max_ = 1;
static constexpr double thrust_min_ = -4.064121;

int input_mode_ = 1;
int q_position_x_ = 60;
int q_position_y_ = 60;
int q_position_z_ = 60;
int q_joystick_x_ = 20;
int q_joystick_y_ = 20;
int q_joystick_z_ = 20;
float roll_gain_ = 5.0; //0.9Tmsg
float pitch_gain_ = 5.0; //0.9
float roll_time_constant_ = 0.35; //0.25
float pitch_time_constant_ = 0.35; //0.255
float r_delta_roll_ = 1.0; //0.255
float r_delta_pitch_ = 1.0; //0.255
float r_delta_thrust_ = 1.0; //0.255

class LinearModelPredictiveControl : public rclcpp :: Node {
private:
        //Subscribers and publisher
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_position_sub_;
        rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_command_sub_;
        rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
        rclcpp::Subscription<mav_msgs::msg::Joystick>::SharedPtr joystick_sub_;

        rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr predict_state_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr predict_state_vis_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odom_marker_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_publisher_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

        //messages declarations
        px4_msgs::msg::VehicleOdometry current_odometry_msg_;
        px4_msgs::msg::VehicleAttitudeSetpoint command_attitude_thrust_;
        Eigen::Vector3d current_position_W; 
        Eigen::Vector3d current_yrp_W; //current yaw roll pitch ()
        Eigen::Vector2d roll_pitch_inertial_frame_;
        Eigen::Vector3d current_velocity_W; 
        trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msgs_;
        Eigen::Vector3d linearized_command_roll_pitch_thrust_;
        Eigen::Vector4d command_roll_pitch_yaw_thrust_;
        Eigen::Vector3d eular_angles_ref_;
        Eigen::Matrix<double, 3, 1> position_ref, velocity_ref;
        Eigen::Matrix<double, kMeasurementSize, 1> references_;
        Eigen::Matrix<double, kStateSize, 1> joystick_references_;
        Eigen::Matrix<double, kStateSize, 1> current_state_;
        Eigen::Matrix<double, kStateSize, 1> target_state_;
        Eigen::Matrix<double, kInputSize, 1> target_input_;
        Eigen::Vector3d position_prev_;
        Eigen::Matrix<double, kInputSize, 1> u_prev_;
        Eigen::Matrix<double, 4, 1> quaternion_ref;
        Quaternionf q_;

        //system model matrix
        Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   
        Eigen::Matrix<double, kStateSize, kInputSize> model_B_;
        double K_yaw_;
        double yaw_ref_;
        float joystick_x_;
        float joystick_y_;
        bool initialized_ = false;
        unsigned long offboard_setpoint_counter_;
        unsigned long target_marker_countdown_;

        Eigen::Matrix<double, kStateSize + kInputSize, kStateSize + kMeasurementSize> pseudo_inverse_left_hand_side_;

        MpcQueue mpc_queue_;
        std::deque<Eigen::Matrix<double, kStateSize, 1>> CVXGEN_queue_;
        std::deque<Eigen::Matrix<double, kStateSize, 1>> CVXGEN_queue_joystick_;
        
        double time_;
        rclcpp::Time time_prev_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::atomic<uint64_t> timestamp_;
        
        
public:
        LinearModelPredictiveControl() : 
                Node("LinearModelPredictiveControl"),
                u_prev_(0.0, 0.0, 0.0),
                mpc_queue_(kPredictionHorizonSteps),
                joystick_x_(0.00000),
                joystick_y_(0.00000)
        {
                target_marker_countdown_ = 0;
                initialize_parameters();
                tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
                make_transforms();
                attitude_setpoint_pub_ = 
                        this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/vehicle_attitude_setpoint/in", 1);
                offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
                predict_state_vis_publisher_ =
			this->create_publisher<geometry_msgs::msg::PoseArray>("sc/predictive_state_vis", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
                predict_state_publisher_ =
			this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("sc/predict_state", 10);
                odom_publisher_ = 
                        this->create_publisher<geometry_msgs::msg::PoseStamped>("sc/vehicle_odom_vis",10);
                odom_marker_publisher_ = 
                        this->create_publisher<visualization_msgs::msg::Marker>("sc/vehicle_odom_marker",10);
                
                
                timesync_sub_ =
			this->create_subscription<Timesync>("fmu/timesync/out", 10,
				[this](const Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
                joystick_sub_ = 
                        this -> create_subscription<mav_msgs::msg::Joystick>("/sc/joystick_input", 10,
				[this](const mav_msgs::msg::Joystick::SharedPtr msg) {
					if(abs(msg -> lx) > 0.1) joystick_x_ = msg->lx; 
					else joystick_x_ = 0.00000;
					if(abs(msg -> ly) > 0.1) joystick_y_ = -msg->ly;
					else joystick_y_ = 0.00000;
					return;
				});

                vehicle_position_sub_ = 
                	this -> create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out", 10, 
                                [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
                                        Eigen::Vector4d quaternion;
                                        quaternion << msg -> q[0], msg -> q[1], msg -> q[2], msg -> q[3];
                                        getEulerAnglesFromQuaternion(quaternion,&current_yrp_W);
                                        double roll_W = current_yrp_W(2);
                                        double pitch_W = -current_yrp_W(1);
                                        double yaw_W = current_yrp_W(3);
                                        roll_pitch_inertial_frame_ << -sin(yaw_W) * pitch_W + cos(yaw_W) * roll_W, cos(yaw_W) * pitch_W + sin(yaw_W) * roll_W;
                                        current_position_W << msg -> x, msg -> y, msg -> z;
                                        current_velocity_W << msg -> vx, msg -> vy, msg -> vz;
                                        current_state_ << current_position_W, current_velocity_W, roll_pitch_inertial_frame_;
                                        geometry_msgs::msg::PoseStamped pose;
                                        pose.header.frame_id = "map";
                                        pose.header.stamp = this->now();
                                        pose.pose.position.x = current_position_W(1);
                                        pose.pose.position.y = current_position_W(0);
                                        pose.pose.position.z = -current_position_W(2);
                                        pose.pose.orientation.w = quaternion.w();
                                        pose.pose.orientation.x = quaternion.x();
                                        pose.pose.orientation.y = quaternion.y();
                                        pose.pose.orientation.z = quaternion.z();
                                        odom_publisher_->publish(pose);
                        });

                trajectory_command_sub_ = 
                	this -> create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/sc/trajectory_setpoint", 1, 
                                [this](const trajectory_msgs::msg::MultiDOFJointTrajectory::UniquePtr trajectory_msg){
                                        CVXGEN_queue_.clear();
                                        CVXGEN_queue_joystick_.clear();
                                        //摇杆偏移量
                                        MultiDOFJointTrajectoryPoint joystick_point;
                                        joystick_point.transforms.resize(1);
                                        joystick_point.transforms.front().translation.x = trajectory_msg->points[0].transforms.front().translation.x + (joystick_x_ * 5);
                                        joystick_point.transforms.front().translation.y = trajectory_msg->points[0].transforms.front().translation.y + (joystick_y_ * 5);
                                        joystick_point.transforms.front().translation.z = trajectory_msg->points[0].transforms.front().translation.z;
                                        //x_ss[t]赋值
                                        mpc_queue_.fillQueueWithPoint(trajectory_msg->points[0]);
                                        mpc_queue_.fillQueueWithJoystickPoint(joystick_point, input_mode_);
                                        Vector3dDeque position_ref, velocity_ref, acceleration_ref;
                                        std::deque<double> yaw_ref, yaw_rate_ref;
                                        Eigen::Vector4d q;
                                        q <<  trajectory_msg->points.front().transforms.front().rotation.w
                                             ,trajectory_msg->points.front().transforms.front().rotation.x
                                             ,trajectory_msg->points.front().transforms.front().rotation.y
                                             ,trajectory_msg->points.front().transforms.front().rotation.z;
                                        
                                        Eigen::Vector3d angle_ref;
                                        getEulerAnglesFromQuaternion(q, &angle_ref);
                                        yaw_ref_ = angle_ref(2);
                                        Vector3dDeque position_joystick;

                                        mpc_queue_.getQueue(position_ref, velocity_ref, acceleration_ref, yaw_ref, yaw_rate_ref);
                                        mpc_queue_.getJoystickQueue(position_joystick);
                                        //
                                        for(int i = 0; i < kPredictionHorizonSteps - 1; i++){
                                                references_ << position_ref.at(i), velocity_ref.at(i);
                                                computeSteadyState(references_, &target_state_, &target_input_);
                                                CVXGEN_queue_.push_back(target_state_);

                                                joystick_references_ << position_joystick.at(i), 0, 0, 0, 0, 0;
                                                CVXGEN_queue_joystick_.push_back(joystick_references_);

                                                // printf("%d   CVXGEN_queue_joystick_ size: %d\n", i, CVXGEN_queue_joystick_.size());
                                                // printf("params.x_sc[%d]: %f\n", i, *params.x_sc[i + 1]);
                                                if(i == 0){
                                                        Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_ss[i])) = target_input_;
                                                }
                                        }

                                        //将摇杆控制量放入optimization problem
                                        for(int i = 1; i < kPredictionHorizonSteps - 1; i++){
                                                Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_sc[i]), kStateSize, 1) = CVXGEN_queue_joystick_.back();
                                        }
                                        
                                        //计算最后一个x_ss，将所有x_ss放入optimization problem中
                                        references_ << position_ref.at(kPredictionHorizonSteps - 1), velocity_ref.at(kPredictionHorizonSteps - 1);
                                        // computeSteadyState(references_, &target_state_, &target_input_);
                                        target_state_ << references_, 0.000000, 0.000000;
                                        CVXGEN_queue_.push_back(target_state_);

                                        for (int i = 0; i < kPredictionHorizonSteps; i++){
                                                Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_ss[i]), kStateSize, 1) = CVXGEN_queue_[i];
                                        }
                                });
                        model_A_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(&params.A[0], kStateSize, kStateSize);
                        model_B_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(&params.B[0], kStateSize, kInputSize);
                        InitializeSteadyStateSolver(model_A_, model_B_);

                        auto timer_callback = [this]() -> void {
                                //RCLCPP_INFO(this->get_logger(),"Timer callback:");
                	        if (offboard_setpoint_counter_ == 20) {
				        // Change to Offboard mode after 10 setpoints
				        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				        // Arm the vehicle
				        this->arm();
			        }
                                get_parameters();
            		        // offboard_control_mode needs to be paired with trajectory_setpoint
                                ComputeRollPitchThrustCommand();
                                publishPredictState();
                                publishOdometryMarker();
			        publish_offboard_control_mode();
			        publish_attitude_setpoint();

                                if(offboard_setpoint_counter_ >= 11){
                                        static int counter = 0;
                                        if(counter >= 0){
                                                std::cout << "--------------------------------------------------------------------------------" << std::endl;
                                                RCLCPP_INFO(this->get_logger(),"Odometry messages received: \n\t\t\t\t\t\t\t\tpos_x: %g\tpos_y: %g\tpos_z: %g\n\t\t\t\t\t\t\t\tvel_x: %g\tvel_y: %g\tvel_z: %g\n\t\t\t\t\t\t\t\troll: %g\tpitch: %g\tyaw: %g",
                                                                current_position_W(0),current_position_W(1),current_position_W(2),
                                                                current_velocity_W(0),current_velocity_W(1),current_velocity_W(2), 
                                                                current_yrp_W(1),current_yrp_W(2),current_yrp_W(0));
                                                RCLCPP_INFO(this->get_logger(), "Trajectory messages received, set state references: \n\t\t\t\t\t\t\t\tpos_x: %g\tpos_y: %g\tpos_z: %g\n\t\t\t\t\t\t\t\tjoystick_x: %g\tjoystick_y: %g\tjoystick_z: %g\n\t\t\t\t\t\t\t\tvel_x: %g\tvel_y: %g\tvel_z: %g\n\t\t\t\t\t\t\t\troll_ref: %g\tpitch_ref: %g", 
                                                                      params.x_ss_0[0],params.x_ss_0[1],params.x_ss_0[2],
                                                                      params.x_sc_1[0],params.x_sc_1[1],params.x_sc_1[2],
                                                                      params.x_ss_0[3],params.x_ss_0[4],params.x_ss_0[5],
                                                                      params.x_ss_0[6],params.x_ss_0[7]);
                                                RCLCPP_INFO(this->get_logger(), "previous input: roll_prev: %g\tpitch: %g\tthrust: %g", params.u_prev[0], params.u_prev[1], params.u_prev[2]);
                                                RCLCPP_INFO(this->get_logger(), "current input: roll: %g\tpitch: %g\tthrust: %g", command_roll_pitch_yaw_thrust_(0), command_roll_pitch_yaw_thrust_(1), command_roll_pitch_yaw_thrust_(3));
                                                RCLCPP_INFO(this->get_logger(), "Quaternion:   %.6f   %.6f   %.6f   %.6f", q_.coeffs()(0),q_.coeffs()(1),q_.coeffs()(2),q_.coeffs()(3));
                                                counter = 0;
                                        }
                                        counter++;
                                }

           		        // stop the counter after reaching 11
			        if (offboard_setpoint_counter_ < 11) {
				        offboard_setpoint_counter_++;
			        }
                	};
                timer_ = this -> create_wall_timer(20ms, timer_callback);
        };

        //void set_reference(const trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msg);
        /**
         * @brief Get the Euler Angles From Quaternion object
         * 
         * @param q 
         * @param euler_angles(roll, pitch, yaw) 
         */
        void inline getEulerAnglesFromQuaternion(const Eigen::Vector4d q, Eigen::Vector3d* euler_angles)
        {
                assert(euler_angles != NULL);
                *euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
                        std::asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
                        std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        }
        void inline getQuaternionFromEulerAngle(Quaternionf &q, const float roll, const float pitch, const float yaw)
        {
                q = AngleAxisf(roll, Vector3f::UnitX())       
                        * AngleAxisf(pitch, Vector3f::UnitY())
                        * AngleAxisf(yaw, Vector3f::UnitZ());
        }
        void publishRPYTCommand();
        void arm();
        void publish_offboard_control_mode() const;
	void publish_attitude_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
        void InitializeSteadyStateSolver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
        void computeSteadyState(const Eigen::Matrix<double, kMeasurementSize, 1> &reference,
                                Eigen::Matrix<double, kStateSize, 1>* steadystate_state,
                                Eigen::Matrix<double, kInputSize, 1>* steadystate_input);
        void ComputeRollPitchThrustCommand();
        Eigen::Vector3d calculateVelocity(const Eigen::Vector3d &current_position, const Eigen::Vector3d &previous_position);
        void publishPredictState();
        void publishOdometryMarker();
        void make_transforms();
        void initialize_parameters();
        void get_parameters();
};

}
