#include <px4_ros_com/mpc_queue.hpp>
#define INPUT_MODEL 1
using namespace shared_control_mpc;
/**
 * @brief 将传输过来的轨迹点设为预测区间内所有的参考轨迹点
 * 
 * @param point 
 */
void MpcQueue::fillQueueWithPoint(MultiDOFJointTrajectoryPoint &point){
        clearQueue();
        while(current_queue_size_ < max_queue_size_){
                Eigen::Vector3d position(point.transforms.front().translation.x, point.transforms.front().translation.y, point.transforms.front().translation.z);
                pos_ref_.push_back(position);
                Eigen::Vector3d velocity(point.velocities.front().linear.x, point.velocities.front().linear.y, point.velocities.front().linear.z);
                vel_ref_.push_back(velocity);
                Eigen::Vector3d acceleration(point.accelerations.front().linear.x, point.accelerations.front().linear.y, point.accelerations.front().linear.z);
                acc_ref_.push_back(acceleration);
                yaw_ref_.push_back(0);
                yaw_rate_ref_.push_back(0);
                current_queue_size_++;
        }
}
void MpcQueue::fillQueueWithJoystickPoint(MultiDOFJointTrajectoryPoint &point, int mode){
        switch (mode)
        {
        case 1:
                constantInputModel(point);
                break;
        case 2:
                linearInputModel(point);
                break;
        case 3:
                quadraticInputModel(point);
                break;
        }
}

void MpcQueue::constantInputModel(MultiDOFJointTrajectoryPoint &point){
        clearJoystickQueue();
        while(current_joystick_queue_size_ < max_queue_size_){
                Eigen::Vector3d position(point.transforms.front().translation.x, point.transforms.front().translation.y, point.transforms.front().translation.z);
                pos_joystick_.push_back(position);
                current_joystick_queue_size_++;
        }
}

void MpcQueue::linearInputModel(MultiDOFJointTrajectoryPoint &point){
        clearJoystickQueue();
        float increment_x = point.transforms.front().translation.x / (float)max_queue_size_;
        float increment_y = point.transforms.front().translation.y / (float)max_queue_size_;
        float x = point.transforms.front().translation.x;
        float y = point.transforms.front().translation.y;
        while(current_joystick_queue_size_ < max_queue_size_){
                Eigen::Vector3d position(x - increment_x * current_queue_size_, y - increment_y * current_queue_size_, point.transforms.front().translation.z);
                pos_joystick_.push_back(position);
                current_joystick_queue_size_++;
        }
}
/**
 * @brief 计算二次函数模型的输出
 * y = ax^2 + bx + c 
 * @param point 
 */
void MpcQueue::quadraticInputModel(MultiDOFJointTrajectoryPoint &point){
        clearJoystickQueue();
        float a_x =  point.transforms.front().translation.x * (-4) / (float)(max_queue_size_ * max_queue_size_);
        float b_x = -a_x * max_queue_size_; 
        float a_y =  point.transforms.front().translation.y * (-4) / (float)(max_queue_size_ * max_queue_size_);
        float b_y = -a_y * max_queue_size_; 
        while(current_joystick_queue_size_ < max_queue_size_){
                Eigen::Vector3d position(a_x * pow(current_queue_size_ , 2) + b_x * current_queue_size_, a_y * pow(current_queue_size_ , 2) + b_y * current_queue_size_, point.transforms.front().translation.z);
                pos_joystick_.push_back(position);
                current_joystick_queue_size_++;
        }
}
/**
 * @brief 清理类内成员
 * 
 */
void MpcQueue::clearQueue(){
        pos_ref_.clear();
        vel_ref_.clear();
        acc_ref_.clear();
        yaw_ref_.clear();
        yaw_rate_ref_.clear();
        current_queue_size_ = 0;
}

void MpcQueue::clearJoystickQueue(){
        pos_joystick_.clear();
        current_joystick_queue_size_ = 0;
}
/**
 * @brief 将mpc_queue类内成员取出放入实参内
 * 
 * @param position_reference 
 * @param velocity_reference 
 * @param acceleration_reference 
 * @param yaw_reference 
 * @param yaw_rate_reference 
 */
void MpcQueue::getQueue(Vector3dDeque& position_reference, Vector3dDeque& velocity_reference,
                        Vector3dDeque& acceleration_reference, std::deque<double>& yaw_reference,
                        std::deque<double>& yaw_rate_reference){
        position_reference.clear();
	velocity_reference.clear();
	acceleration_reference.clear();
	yaw_reference.clear();
	yaw_rate_reference.clear();

        for(int i=0; i < max_queue_size_; i++){
	        position_reference.push_back(pos_ref_.at(i));
	        velocity_reference.push_back(vel_ref_.at(i));
	        acceleration_reference.push_back(acc_ref_.at(i));
	        yaw_reference.push_back(yaw_ref_.at(i));
	        yaw_rate_reference.push_back(yaw_rate_ref_.at(i));
        }
}
void MpcQueue::getJoystickQueue(Vector3dDeque& position_joystick){
        position_joystick.clear();
        for(int i=0; i < max_queue_size_; i++){
	        position_joystick.push_back(pos_joystick_.at(i));
        }
}


