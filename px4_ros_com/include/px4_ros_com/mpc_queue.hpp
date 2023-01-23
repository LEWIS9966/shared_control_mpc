#include <bits/stdc++.h>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

using namespace trajectory_msgs::msg;
typedef std::deque<Eigen::Vector3d> Vector3dDeque;


namespace shared_control_mpc{

typedef std::deque<Eigen::Vector3d> Vector3dDeque;

class MpcQueue : public rclcpp::Node{
public:
MpcQueue(int mpc_queue_size):
    Node("mpc_queue"),
    max_queue_size_(mpc_queue_size),
    current_queue_size_(0)
    {
        RCLCPP_INFO(this->get_logger(), "mpc_queue initialized");
    };
~MpcQueue(){};
void fillQueueWithPoint(MultiDOFJointTrajectoryPoint &point);
void fillQueueWithJoystickPoint(MultiDOFJointTrajectoryPoint &point, int mode);
void constantInputModel(MultiDOFJointTrajectoryPoint &point);
void linearInputModel(MultiDOFJointTrajectoryPoint &point);
void quadraticInputModel(MultiDOFJointTrajectoryPoint &point);
void clearQueue();
void clearJoystickQueue();
void getQueue(Vector3dDeque& position_reference, Vector3dDeque& velocity_reference,
                        Vector3dDeque& acceleration_reference, std::deque<double>& yaw_reference,
                        std::deque<double>& yaw_rate_reference);
void getJoystickQueue(Vector3dDeque& position_joystick);


private:
int max_queue_size_;
int current_queue_size_;
int current_joystick_queue_size_;
Vector3dDeque pos_ref_, pos_joystick_;
Vector3dDeque vel_ref_;
Vector3dDeque acc_ref_;
std::deque<double> yaw_ref_;
std::deque<double> yaw_rate_ref_;
};
}