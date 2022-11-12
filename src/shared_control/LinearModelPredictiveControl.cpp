#include <px4_ros_com/LinearModelPredictiveControl.hpp>
#include <fstream>
#define MARKER_KEEPLAST 700


using namespace shared_control_mpc;
Vars vars;
Params params;
Workspace work;
Settings settings;
Eigen::Matrix<double, kStateSize, kStateSize> model_A;   //dynamics matrix
Eigen::Matrix<double, kStateSize, kInputSize> model_B;   //transfer matrix
Eigen::Matrix<double, kStateSize, kInputSize> model_Bd; 
Eigen::MatrixXd LQR_K;


/**
 * @brief 
 * set penalty matrix and  #ifndef ENABLE_LQR input limits
 */
void load_data_penality(){
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;

  Eigen::Vector3d q_joystick_position;
  Eigen::Vector3d q_joystick_velocity;
  Eigen::Vector3d q_joystick_attitude;

  Eigen::Vector3d r_command;
  Eigen::Vector3d r_delta_command;

  q_position << q_position_x_, q_position_y_, q_position_z_;
  q_velocity << 20, 20, 25;
  q_attitude << 20, 20;

  q_joystick_position << q_joystick_x_, q_position_y_, q_position_z_;
  q_joystick_velocity << 0, 0, 0;
  q_joystick_attitude << 0, 0;

  r_command << 35, 35, 2;
  r_delta_command << r_delta_roll_, r_delta_pitch_, r_delta_thrust_;

  Eigen::Matrix<double, kStateSize, kStateSize> Q;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_joystick; 
  Eigen::Matrix<double, kInputSize, kInputSize> R;
  Eigen::Matrix<double, kInputSize, kInputSize> R_delta;

  Q.setZero();
  Q_final.setZero();
  Q_joystick.setZero();
  R.setZero();
  R_delta.setZero();

  Q.block(0, 0, 3, 3) = q_position.asDiagonal();
  Q.block(3, 3, 3, 3) = q_velocity.asDiagonal();
  Q.block(6, 6, 2, 2) = q_attitude.asDiagonal();
  Q_joystick.block(0, 0, 3, 3) = q_joystick_position.asDiagonal();
  Q_joystick.block(3, 3, 3, 3) = q_joystick_velocity.asDiagonal();
  Q_joystick.block(6, 6, 2, 2) = q_joystick_attitude.asDiagonal();

  R = r_command.asDiagonal();

  R_delta = r_delta_command.asDiagonal();

  // //Compute terminal cost
  // //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;

  Q_final = Q;
  for (int i = 0; i < 1000; i++) {
    Eigen::MatrixXd temp = (model_B.transpose() * Q_final * model_B + R);
    Q_final = model_A.transpose() * Q_final * model_A - (model_A.transpose() * Q_final * model_B) * temp.inverse() 
                * (model_B.transpose() * Q_final * model_A) + Q;
  }
  //lqr matrix
  Eigen::MatrixXd temporary_matrix = model_B.transpose() * Q_final * model_B + R;
  LQR_K = temporary_matrix.inverse() * (model_B.transpose() * Q_final * model_A);

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_x), kStateSize, kStateSize) = Q;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_x_sc), kStateSize, kStateSize) = Q_joystick;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.P), kStateSize, kStateSize) = Q_final;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_u), kInputSize, kInputSize) = R;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_delta), kInputSize, kInputSize) = R_delta * (1.0 / sampling_time_ * sampling_time_);

  params.u_max[0] = roll_limit_;
  params.u_max[1] = pitch_limit_;
  params.u_max[2] = thrust_max_;

  params.u_min[0] = -roll_limit_;
  params.u_min[1] = -pitch_limit_;
  params.u_min[2] = thrust_min_;

  printf("Linear MPC: Tuning parameters updated...\n");
  printf("Linear MPC: Q:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kStateSize; ++j){
      std::cout << Q(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("Linear MPC: Q_x_sc:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kStateSize; ++j){
      std::cout << Q_joystick(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLinear MPC: Q_final:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kStateSize; ++j){
      std::cout << Q_final(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLinear MPC: R:\n");
  for(int i = 0; i < kInputSize; ++i){
    for(int j = 0; j < kInputSize; ++j){
      std::cout << R(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLinear MPC: R_delta:\n");
  for(int i = 0; i < kInputSize; ++i){
    for(int j = 0; j < kInputSize; ++j){
      std::cout << R_delta(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLQR: K:\n");
  for(int i = 0; i < LQR_K.rows(); ++i){
    for(int j = 0; j < LQR_K.cols(); ++j){
      std::cout << LQR_K(i,j) << "\t";
    }
    std::cout << std::endl;
  }
}


/**
 * @brief 
 * set model matrix
 */
void load_data_model() {
  // In this function, load all problem instance data.
  std::vector<double> drag_coefficients = {0.0, 0.0, 0.0000};

  Eigen::MatrixXd A_continous_time(kStateSize, kStateSize);
  A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);

  Eigen::MatrixXd B_continous_time;
  B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);

  Eigen::MatrixXd Bd_continous_time;
  Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);     

  A_continous_time(0, 3) = 1;
  A_continous_time(1, 4) = 1;
  A_continous_time(2, 5) = 1;
  A_continous_time(3, 3) = -drag_coefficients.at(0);
  A_continous_time(3, 7) = -kGravity;
  A_continous_time(4, 4) = -drag_coefficients.at(1);
  A_continous_time(4, 6) = kGravity;
  A_continous_time(5, 5) = -drag_coefficients.at(2);
  A_continous_time(6, 6) = -1.0 / roll_time_constant_;
  A_continous_time(7, 7) = -1.0 / pitch_time_constant_;    
  
  B_continous_time(5, 2) = 1;
  B_continous_time(6, 0) = roll_gain_ / roll_time_constant_;
  B_continous_time(7, 1) = pitch_gain_ / pitch_time_constant_;  

  Bd_continous_time(3, 0) = 1.0;
  Bd_continous_time(4, 1) = 1.0;
  Bd_continous_time(5, 2) = 1.0;   

  model_A = (prediction_sampling_time_ * A_continous_time).exp(); 

  Eigen::MatrixXd integral_exp_A;
  integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  const int count_integral_A = 100;    
  
  for (int i = 0; i < count_integral_A; i++) {
    integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
        * prediction_sampling_time_ / count_integral_A;
  }    
  model_B = integral_exp_A * B_continous_time;
  model_Bd = integral_exp_A * Bd_continous_time;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B;
//  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) = model_Bd_;

  std::cout << "Linear MPC: model matrix initialized correctly" << std::endl;
  printf("Linear MPC: A:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kStateSize; ++j){
      std::cout << model_A(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLinear MPC: B:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kInputSize; ++j){
      std::cout << model_B(i,j) << "\t";
    }
    std::cout << std::endl;
  }
/*  printf("\nLinear MPC: Bd:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kDisturbanceSize; ++j){
      std::cout << model_Bd_(i,j) << "\t";
    }
    std::cout << std::endl;
  }*/
}


/**
 * @brief 
 * 计算控制量roll pitch yaw thrust 
 */
void LinearModelPredictiveControl::ComputeRollPitchThrustCommand(){
  Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_prev)) = u_prev_;
  Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_0)) = current_state_;
  settings.verbose = 0;
  tic();
  solve(); 
  time_ = tocq();
  // linearized_command_roll_pitch_thrust_ = LQR_K * (target_state_ - current_state_);
  // linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMax(Eigen::Vector3d(-roll_limit_, -pitch_limit_, thrust_min_));
  // linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMin(Eigen::Vector3d(roll_limit_, pitch_limit_, thrust_max_));
  // time_ = tocq();
  RCLCPP_INFO(this->get_logger(),"===================Actual time taken: %g seconds.======================", time_);
  double roll = current_yrp_W(1);
  double pitch = current_yrp_W(2);
  double yaw = current_yrp_W(0);
  linearized_command_roll_pitch_thrust_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];

  //T_body_frame = （T_world_frame - g） / (cos(roll) * cos(pitch)), compensate the impact from roll and pitch angle.
  command_roll_pitch_yaw_thrust_(3) = (linearized_command_roll_pitch_thrust_(2) - kGravity) / (cos(roll) * cos(pitch));
  // double pitch_body = linearized_command_roll_pitch_thrust_(1) * (-kGravity / command_roll_pitch_yaw_thrust_(3));
  // double roll_body = linearized_command_roll_pitch_thrust_(0) * (-kGravity / command_roll_pitch_yaw_thrust_(3));
  // command_roll_pitch_yaw_thrust_(0) = pitch_body * sin(yaw) + roll_body * cos(yaw);
  // command_roll_pitch_yaw_thrust_(1) = pitch_body * cos(yaw) - roll_body * sin(yaw);
  command_roll_pitch_yaw_thrust_(0) = linearized_command_roll_pitch_thrust_(0);
  command_roll_pitch_yaw_thrust_(1) = linearized_command_roll_pitch_thrust_(1);
  command_roll_pitch_yaw_thrust_(2) = 0;
  command_roll_pitch_yaw_thrust_(3) = command_roll_pitch_yaw_thrust_(3)/ (-thrust_min_ + kGravity);
  getQuaternionFromEulerAngle(q_, command_roll_pitch_yaw_thrust_(0), command_roll_pitch_yaw_thrust_(1), 0);
  u_prev_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];
  //yaw_rate controller
  double yaw_error = command_roll_pitch_yaw_thrust_(2) - yaw;

  if (std::abs(yaw_error) > M_PI) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * M_PI;
    } else {
      yaw_error = yaw_error + 2.0 * M_PI;
    }
  }

  //double yaw_rate_cmd = K_yaw_ * yaw_error + yaw_rate_ref_.front(); // feed-forward yaw_rate cmd

  // if (yaw_rate_cmd > yaw_rate_limit_) {
  //   yaw_rate_cmd = yaw_rate_limit_;
  // }

  // if (yaw_rate_cmd < -yaw_rate_limit_) {
  //   yaw_rate_cmd = -yaw_rate_limit_;
  // }
  // getQuaternionFromEulerAngle(q_, command_roll_pitch_yaw_thrust_(0), command_roll_pitch_yaw_thrust_(1), 0);

}
/**
 * @brief arm
 * 
 */
void LinearModelPredictiveControl::arm(){
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}
/**
 * @brief 
 * 
 */
void LinearModelPredictiveControl::publish_vehicle_command(uint16_t command, float param1, float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	vehicle_command_publisher_->publish(msg);
}
/**
 * @brief change uavs control mode
 * 
 */
void LinearModelPredictiveControl::publish_offboard_control_mode() const{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief
 * publisher for roll pitch yaw thrust command
 */
void LinearModelPredictiveControl::publish_attitude_setpoint(){
    px4_msgs::msg::VehicleAttitudeSetpoint attitude_msg;
    attitude_msg.timestamp = timestamp_.load();
    attitude_msg.q_d[0] = q_.coeffs()[3];
    attitude_msg.q_d[1] = q_.coeffs()[0];
    attitude_msg.q_d[2] = q_.coeffs()[1];
    attitude_msg.q_d[3] = q_.coeffs()[2];
    attitude_msg.thrust_body[0] = 0;
    attitude_msg.thrust_body[1] = 0;
    attitude_msg.thrust_body[2] = command_roll_pitch_yaw_thrust_(3);
    attitude_setpoint_pub_ -> publish(attitude_msg);
}

/**
 * @brief initialize steady state solver
 * 
 * @param A 
 * @param B 
 */

void LinearModelPredictiveControl::InitializeSteadyStateSolver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  Eigen::MatrixXd left_hand_side;
  left_hand_side.resize(kStateSize + kMeasurementSize, kStateSize + kInputSize);

  Eigen::MatrixXd C(6, 8);
  C.setIdentity();

  left_hand_side << A - Eigen::MatrixXd::Identity(kStateSize, kStateSize), B, C, Eigen::MatrixXd::Zero(
      kMeasurementSize, kInputSize);

  pseudo_inverse_left_hand_side_ = (left_hand_side.transpose() * left_hand_side).inverse()
      * left_hand_side.transpose();
}
/**
 * @brief compute steady state considering disturbance(which is ignored now). 
 *        computed state and input will be put into steadystate_state and steadystate_input
 * @param reference 
 * @param steadystate_state 
 * @param steadystate_input 
 */
void LinearModelPredictiveControl::computeSteadyState(
    const Eigen::Matrix<double, kMeasurementSize, 1> &reference,
    Eigen::Matrix<double, kStateSize, 1>* steadystate_state,
    Eigen::Matrix<double, kInputSize, 1>* steadystate_input)
{

  Eigen::Matrix<double, kStateSize + kInputSize, 1> target_state_and_input;
  Eigen::Matrix<double, kStateSize + kMeasurementSize, 1> right_hand_side;

  right_hand_side << Eigen::MatrixXd::Zero(kStateSize, 1), reference;

  target_state_and_input = pseudo_inverse_left_hand_side_ * right_hand_side;
  *steadystate_state = target_state_and_input.segment(0, kStateSize);
  *steadystate_input = target_state_and_input.segment(kStateSize, kInputSize);
}

/**
 * @brief publish predictied state from MPC solver for visualization
 * 
 */
void LinearModelPredictiveControl::publishPredictState(){
  trajectory_msgs::msg::MultiDOFJointTrajectory predict_state;
  predict_state.header.frame_id = "map";
  predict_state.header.stamp = this->now();
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.header.frame_id = "map";
  poseArray.header.stamp = rclcpp::Clock().now();
  for(int i = 1; i < 6; i++){
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint predictive_point;
    predictive_point.transforms.resize(1);
    predictive_point.transforms.front().translation.x = vars.x[i][0];
    predictive_point.transforms.front().translation.y = vars.x[i][1];
    predictive_point.transforms.front().translation.z = vars.x[i][2];
    Quaternionf q;
    getQuaternionFromEulerAngle(q,vars.x[i][6],vars.x[i][7],yaw_ref_);
    predictive_point.transforms.front().rotation.w = q.w();
    predictive_point.transforms.front().rotation.x = q.x();
    predictive_point.transforms.front().rotation.y = q.y();
    predictive_point.transforms.front().rotation.z = q.z();
    geometry_msgs::msg::Pose pose;
    pose.position.x = vars.x[i][1];
    pose.position.y = vars.x[i][0];
    pose.position.z = -vars.x[i][2];
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    poseArray.poses.push_back(pose);

    predict_state.points.push_back(predictive_point);

  }
  predict_state_vis_publisher_ -> publish(poseArray);
  predict_state_publisher_ -> publish(predict_state);
}

void LinearModelPredictiveControl::publishOdometryMarker(){
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
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.lifetime = rclcpp::Duration(100);
  marker_msg.pose.position.x = current_position_W(1);
  marker_msg.pose.position.y = current_position_W(0);
  marker_msg.pose.position.z = -current_position_W(2);
  odom_marker_publisher_ -> publish(marker_msg);
}

/**
 * @brief publish a static TF for visualization in rviz
 * 
 */  
void LinearModelPredictiveControl::make_transforms()
{
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->now();
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.translation.x = 0;
  t.transform.translation.y = 0;
  t.transform.translation.z = 0;
  tf2::Quaternion q;
  q.setRPY(0,0,0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_static_broadcaster_->sendTransform(t);
}
/**
 * @brief initialize parameters for tuning
 * 
 */
void LinearModelPredictiveControl::initialize_parameters()
{
  input_mode_ = this-> declare_parameter("input_mode", 1);
  q_position_x_ = this-> declare_parameter("q_position_x", 60);
  q_position_y_ = this-> declare_parameter("q_position_y", 60);
  q_position_z_ = this-> declare_parameter("q_position_z", 60);
  q_joystick_x_ = this -> declare_parameter("q_joystick_x", 20);
  q_joystick_y_ = this -> declare_parameter("q_joystick_y", 20);
  q_joystick_z_ = this -> declare_parameter("q_joystick_z", 20);
  r_delta_roll_ = this -> declare_parameter("r_delta_roll", 0.35);
  r_delta_pitch_ = this -> declare_parameter("r_delta_pitch", 0.35);
  r_delta_thrust_ = this -> declare_parameter("r_delta_thrust", 0.35);
  roll_gain_ = this -> declare_parameter("roll_gain", 5.0);
  pitch_gain_ = this -> declare_parameter("pitch_gain", 5.0);
  roll_time_constant_ = this -> declare_parameter("roll_time_constant", 0.35);
  pitch_time_constant_ = this -> declare_parameter("pitch_time_constant", 0.35);
  RCLCPP_INFO(this->get_logger(), "parameters initiallized...");
}
/**
 * @brief get parameter values from ros parameter server
 * 
 */
void LinearModelPredictiveControl::get_parameters()
{
  input_mode_ = this -> get_parameter("input_mode").as_int();
  q_position_x_ = this -> get_parameter("q_position_x").as_int();
  q_position_y_ = this -> get_parameter("q_position_y").as_int();
  q_position_z_ = this -> get_parameter("q_position_z").as_int();
  q_joystick_x_ = this -> get_parameter("q_joystick_x").as_int();
  q_joystick_y_ = this -> get_parameter("q_joystick_y").as_int();
  q_joystick_z_ = this -> get_parameter("q_joystick_z").as_int();
  r_delta_roll_ = this -> get_parameter("r_delta_roll").as_double();;
  r_delta_pitch_ = this -> get_parameter("r_delta_pitch").as_double();;
  r_delta_thrust_ = this -> get_parameter("r_delta_thrust").as_double();;
  roll_gain_ = this -> get_parameter("roll_gain").as_double();
  pitch_gain_ = this -> get_parameter("pitch_gain").as_double();
  roll_time_constant_ = this -> get_parameter("roll_time_constant").as_double();
  pitch_time_constant_ = this -> get_parameter("pitch_time_constant").as_double();
  // RCLCPP_INFO(this->get_logger(),"Parameter:\nK_x: %d K_y: %d K_z: %d K_x_j: %d K_y_j: %d K_z_j: %d r_x_j: %f r_y_j: %f r_t_j: %f roll_gain: %f pitch_gain: %f roll_time_constant: %f pitch_time_constant: %f",
  //             q_position_x_, q_position_y_, q_position_z_, q_joystick_x_, q_joystick_y_, q_joystick_z_, r_delta_roll_, r_delta_pitch_, r_delta_thrust_, roll_gain_, pitch_gain_, roll_time_constant_, pitch_time_constant_);
}
//--------------------------------------------------------------------------------------------------


int main(int argc, char **argv) {
  using namespace shared_control_mpc;
  set_defaults();  // Set basic algorithm parameters.
  setup_indexing();
  load_data_model();
  load_data_penality();
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LinearModelPredictiveControl>());


  /*int num_iters_ ,num_test = 0;
  for(;;){
    num_iters_ = solve();
    use_solution();
  }*/

  /*#if (num_test > 0)
  settings.verbose = 0;
  tic();
  for (int i = 0; i < 20; ++i) {  // Main control loop.
    // Solve our problem at high speed!
    solve();
    // Recommended: check work.converged == 1.
  }
  double time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", num_test, time);
  double time_per = time / num_test;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
  #endif*/
  rclcpp::shutdown();
  return 0;
}