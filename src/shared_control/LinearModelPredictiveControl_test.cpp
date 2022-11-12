#include <px4_ros_com/LinearModelPredictiveControl.hpp>
#include <iostream>

#define SOLVERTEST 0
#define  num_test 0
using namespace shared_control_mpc;
Vars vars;
Params params;
Workspace work;
Settings settings;
Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
Eigen::Matrix<double, kStateSize, kInputSize> model_Bd_; 


void load_data_x_ss(){
  params.x_0[0] = 0;
  params.x_0[1] = 0;
  params.x_0[2] = 0;
  params.x_0[3] = 0;
  params.x_0[4] = 0;
  params.x_0[5] = 0;
  params.x_0[6] = 0;
  params.x_0[7] = 0;
  params.x_ss_0[0] = 5;
  params.x_ss_0[1] = 5;
  params.x_ss_0[2] = -2.5;
  params.x_ss_0[3] = 0.1;
  params.x_ss_0[4] = 0.1;
  params.x_ss_0[5] = 0.1;
  params.x_ss_0[6] = 0.1;
  params.x_ss_0[7] = 0.1;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.u_prev[0] = 0;
  params.u_prev[1] = 0;
  params.u_prev[2] = 0;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  #if(SOLVERTEST)
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  //Q_x
  params.Q_x[0] = 1.0239818823771654;
  params.Q_x[8] = 0;
  params.Q_x[16] = 0;
  params.Q_x[24] = 0;
  params.Q_x[32] = 0;
  params.Q_x[40] = 0;
  params.Q_x[48] = 0;
  params.Q_x[56] = 0;
  params.Q_x[1] = 0;
  params.Q_x[9] = 1.5588540879908819;
  params.Q_x[17] = 0;
  params.Q_x[25] = 0;
  params.Q_x[33] = 0;
  params.Q_x[41] = 0;
  params.Q_x[49] = 0;
  params.Q_x[57] = 0;
  params.Q_x[2] = 0;
  params.Q_x[10] = 0;
  params.Q_x[18] = 1.2592524469074653;
  params.Q_x[26] = 0;
  params.Q_x[34] = 0;
  params.Q_x[42] = 0;
  params.Q_x[50] = 0;
  params.Q_x[58] = 0;
  params.Q_x[3] = 0;
  params.Q_x[11] = 0;
  params.Q_x[19] = 0;
  params.Q_x[27] = 1.4151011970100695;
  params.Q_x[35] = 0;
  params.Q_x[43] = 0;
  params.Q_x[51] = 0;
  params.Q_x[59] = 0;
  params.Q_x[4] = 0;
  params.Q_x[12] = 0;
  params.Q_x[20] = 0;
  params.Q_x[28] = 0;
  params.Q_x[36] = 1.2835250817713186;
  params.Q_x[44] = 0;
  params.Q_x[52] = 0;
  params.Q_x[60] = 0;
  params.Q_x[5] = 0;
  params.Q_x[13] = 0;
  params.Q_x[21] = 0;
  params.Q_x[29] = 0;
  params.Q_x[37] = 0;
  params.Q_x[45] = 1.6931379183129964;
  params.Q_x[53] = 0;
  params.Q_x[61] = 0;
  params.Q_x[6] = 0;
  params.Q_x[14] = 0;
  params.Q_x[22] = 0;
  params.Q_x[30] = 0;
  params.Q_x[38] = 0;
  params.Q_x[46] = 0;
  params.Q_x[54] = 1.4404537176707395;
  params.Q_x[62] = 0;
  params.Q_x[7] = 0;
  params.Q_x[15] = 0;
  params.Q_x[23] = 0;
  params.Q_x[31] = 0;
  params.Q_x[39] = 0;
  params.Q_x[47] = 0;
  params.Q_x[55] = 0;
  params.Q_x[63] = 1.1568677384749633;
//R_delta
  params.R_delta[0] = 1.8457508712931792;
  params.R_delta[3] = 0;
  params.R_delta[6] = 0;
  params.R_delta[1] = 0;
  params.R_delta[4] = 1.3779940413288891;
  params.R_delta[7] = 0;
  params.R_delta[2] = 0;
  params.R_delta[5] = 0;
  params.R_delta[8] = 1.0922170088717242;
//R_u
  params.R_u[0] = 1.2219578839321814;
  params.R_u[3] = 0;
  params.R_u[6] = 0;
  params.R_u[1] = 0;
  params.R_u[4] = 1.3879712575556487;
  params.R_u[7] = 0;
  params.R_u[2] = 0;
  params.R_u[5] = 0;
  params.R_u[8] = 1.9363836498604305;
//P
  params.P[0] = 1.0420308509935736;
  params.P[8] = 0;
  params.P[16] = 0;
  params.P[24] = 0;
  params.P[32] = 0;
  params.P[40] = 0;
  params.P[48] = 0;
  params.P[56] = 0;
  params.P[1] = 0;
  params.P[9] = 1.4172668947572267;
  params.P[17] = 0;
  params.P[25] = 0;
  params.P[33] = 0;
  params.P[41] = 0;
  params.P[49] = 0;
  params.P[57] = 0;
  params.P[2] = 0;
  params.P[10] = 0;
  params.P[18] = 1.0042664341671594;
  params.P[26] = 0;
  params.P[34] = 0;
  params.P[42] = 0;
  params.P[50] = 0;
  params.P[58] = 0;
  params.P[3] = 0;
  params.P[11] = 0;
  params.P[19] = 0;
  params.P[27] = 1.2465354688608894;
  params.P[35] = 0;
  params.P[43] = 0;
  params.P[51] = 0;
  params.P[59] = 0;
  params.P[4] = 0;
  params.P[12] = 0;
  params.P[20] = 0;
  params.P[28] = 0;
  params.P[36] = 1.7060561835840065;
  params.P[44] = 0;
  params.P[52] = 0;
  params.P[60] = 0;
  params.P[5] = 0;
  params.P[13] = 0;
  params.P[21] = 0;
  params.P[29] = 0;
  params.P[37] = 0;
  params.P[45] = 1.0615407159206998;
  params.P[53] = 0;
  params.P[61] = 0;
  params.P[6] = 0;
  params.P[14] = 0;
  params.P[22] = 0;
  params.P[30] = 0;
  params.P[38] = 0;
  params.P[46] = 0;
  params.P[54] = 1.2946934986032799;
  params.P[62] = 0;
  params.P[7] = 0;
  params.P[15] = 0;
  params.P[23] = 0;
  params.P[31] = 0;
  params.P[39] = 0;
  params.P[47] = 0;
  params.P[55] = 0;
  params.P[63] = 1.988112752812178;
//A
  params.A[0] = 1.884888920907902;
  params.A[1] = -0.0726144452811801;
  params.A[2] = 0.9427735461129836;
  params.A[3] = 0.5306230967445558;
  params.A[4] = -0.1372277142250531;
  params.A[5] = 1.4282657305652786;
  params.A[6] = -1.309926991335284;
  params.A[7] = 1.3137276889764422;
  params.A[8] = -1.8317219061667278;
  params.A[9] = 1.4678147672511939;
  params.A[10] = 0.703986349872991;
  params.A[11] = -0.2163435603565258;
  params.A[12] = 0.6862809905371079;
  params.A[13] = -0.15852598444303245;
  params.A[14] = 1.1200128895143409;
  params.A[15] = -1.5462236645435308;
  params.A[16] = 0.0326297153944215;
  params.A[17] = 1.4859581597754916;
  params.A[18] = 1.71011710324809;
  params.A[19] = -1.1186546738067493;
  params.A[20] = -0.9922787897815244;
  params.A[21] = 1.6160498864359547;
  params.A[22] = -0.6179306451394861;
  params.A[23] = -1.7725097038051376;
  params.A[24] = 0.8595466884481313;
  params.A[25] = -0.3423245633865686;
  params.A[26] = 0.9412967499805762;
  params.A[27] = -0.09163346622652258;
  params.A[28] = 0.002262217745727657;
  params.A[29] = -0.3297523583656421;
  params.A[30] = -0.8380604158593941;
  params.A[31] = 1.6028434695494038;
  params.A[32] = 0.675150311940429;
  params.A[33] = 1.1553293733718686;
  params.A[34] = 1.5829581243724693;
  params.A[35] = -0.9992442304425597;
  params.A[36] = 1.6792824558896897;
  params.A[37] = 1.4504203490342324;
  params.A[38] = 0.02434104849994556;
  params.A[39] = 0.27160869657612263;
  params.A[40] = -1.5402710478528858;
  params.A[41] = 1.0484633622310744;
  params.A[42] = -1.3070999712627054;
  params.A[43] = 0.13534416402363814;
  params.A[44] = -1.4942507790851232;
  params.A[45] = -1.708331625671371;
  params.A[46] = 0.436109775042258;
  params.A[47] = -0.03518748153727991;
  params.A[48] = 0.6992397389570906;
  params.A[49] = 1.1634167322171374;
  params.A[50] = 1.9307499705822648;
  params.A[51] = -1.6636772756932747;
  params.A[52] = 0.5248484497343218;
  params.A[53] = 0.30789958152579144;
  params.A[54] = 0.602568707166812;
  params.A[55] = 0.17271781925751872;
  params.A[56] = 0.2294695501208066;
  params.A[57] = 1.4742185345619543;
  params.A[58] = -0.1919535345136989;
  params.A[59] = 0.13990231452144553;
  params.A[60] = 0.7638548150610602;
  params.A[61] = -1.6420200344195646;
  params.A[62] = -0.27229872445076087;
  params.A[63] = -1.5914631171820468;
//B
  params.B[0] = -1.4487604283558668;
  params.B[1] = -1.991497766136364;
  params.B[2] = -1.1611742553535152;
  params.B[3] = -1.133450950247063;
  params.B[4] = 0.06497792493777155;
  params.B[5] = 0.28083295396097263;
  params.B[6] = 1.2958447220129887;
  params.B[7] = -0.05315524470737154;
  params.B[8] = 1.5658183956871667;
  params.B[9] = -0.41975684089933685;
  params.B[10] = 0.97844578833777;
  params.B[11] = 0.2110290496695293;
  params.B[12] = 0.4953003430893044;
  params.B[13] = -0.9184320124667495;
  params.B[14] = 1.750380031759156;
  params.B[15] = 1.0786188614315915;
  params.B[16] = -1.4176198837203735;
  params.B[17] = 0.149737479778294;
  params.B[18] = 1.9831452222223418;
  params.B[19] = -1.8037746699794734;
  params.B[20] = -0.7887206483295461;
  params.B[21] = 0.9632534854086652;
  params.B[22] = -1.8425542093895406;
  params.B[23] = 0.986684363969033;
  //Bd
  params.Bd[0] = 0.2936851199350441;
  params.Bd[1] = 0.9268227022482662;
  params.Bd[2] = 0.20333038350653299;
  params.Bd[3] = 1.7576139132046351;
  params.Bd[4] = -0.614393188398918;
  params.Bd[5] = 0.297877839744912;
  params.Bd[6] = -1.796880083990895;
  params.Bd[7] = 0.21373133661742738;
  params.Bd[8] = -0.32242822540825156;
  params.Bd[9] = 1.9326471511608059;
  params.Bd[10] = 1.7824292753481785;
  params.Bd[11] = -1.4468823405675986;
  params.Bd[12] = -1.8335374338761512;
  params.Bd[13] = -1.5172997317243713;
  params.Bd[14] = -1.229012129120719;
  params.Bd[15] = 0.9046719772422094;
  params.Bd[16] = 0.17591181415489432;
  params.Bd[17] = 0.13970133814112584;
  params.Bd[18] = -0.14185208214985234;
  params.Bd[19] = -1.9732231264739348;
  params.Bd[20] = -0.4301123458221334;
  params.Bd[21] = 1.9957537650387742;
  params.Bd[22] = 1.2811648216477893;
  params.Bd[23] = 0.2914428437588219;
//input limit
  params.u_min[0] = 0.47730909231793106;
  params.u_min[1] = -1.187569373035299;
  params.u_min[2] = -0.6877370247915531;
  params.u_max[0] = -0.6201861482616171;
  params.u_max[1] = -0.4209925183921568;
  params.u_max[2] = -1.9110724537712471;
#endif
}
#if(!SOLVERTEST)
void load_data_penality(){
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;

  Eigen::Vector3d r_command;
  Eigen::Vector3d r_delta_command;
  Eigen::VectorXd control_limits(5);

  q_position << 40, 40, 60;
  q_velocity << 20, 20, 25;
  q_attitude << 20, 20;

  r_command << 35, 35, 2;
  r_delta_command << 0.3, 0.3, 0.0025;

  control_limits << 0.4363323, 0.4363323, 1.5707963, 5.0, 20.0;

  Eigen::Matrix<double, kStateSize, kStateSize> Q;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
  Eigen::Matrix<double, kInputSize, kInputSize> R;
  Eigen::Matrix<double, kInputSize, kInputSize> R_delta;

  Q.setZero();
  Q_final.setZero();
  R.setZero();
  R_delta.setZero();

  Q.block(0, 0, 3, 3) = q_position.asDiagonal();
  Q.block(3, 3, 3, 3) = q_velocity.asDiagonal();
  Q.block(6, 6, 2, 2) = q_attitude.asDiagonal();

  R = r_command.asDiagonal();

  R_delta = r_delta_command.asDiagonal();

  //Compute terminal cost
  //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;

  Q_final = Q;
  for (int i = 0; i < 1000; i++) {
    Eigen::MatrixXd temp = (model_B_.transpose() * Q_final * model_B_ + R);
    Q_final = model_A_.transpose() * Q_final * model_A_ - (model_A_.transpose() * Q_final * model_B_) * temp.inverse() 
                * (model_B_.transpose() * Q_final * model_A_) + Q;
  }
  Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
  //LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_x), kStateSize, kStateSize) = Q;
//  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.P), kStateSize, kStateSize) = Q_final;
//  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_u), kInputSize, kInputSize) = R;
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
  for(int i = 0; i < kInputSize * kInputSize; ++i){
    std::cout << params.R_delta[i] << "\t";
  }
  std::cout << std::endl;
}
void load_data_model() {
  // In this function, load all problem instance data.
  std::vector<double> drag_coefficients = {0.010000, 0.010000, 0.0000};

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
  A_continous_time(3, 7) = kGravity;
  A_continous_time(4, 4) = -drag_coefficients.at(1);
  A_continous_time(4, 6) = -kGravity;
  A_continous_time(5, 5) = -drag_coefficients.at(2);
  A_continous_time(6, 6) = -1.0 / roll_time_constant_;
  A_continous_time(7, 7) = -1.0 / pitch_time_constant_;    
  B_continous_time(5, 2) = 1.0;
  B_continous_time(6, 0) = roll_gain_ / roll_time_constant_;
  B_continous_time(7, 1) = pitch_gain_ / pitch_time_constant_;     
  Bd_continous_time(3, 0) = 1.0;
  Bd_continous_time(4, 1) = 1.0;
  Bd_continous_time(5, 2) = 1.0;   

  model_A_ = (prediction_sampling_time_ * A_continous_time).exp(); 

  Eigen::MatrixXd integral_exp_A;
  integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  const int count_integral_A = 100;    
  
  for (int i = 0; i < count_integral_A; i++) {
    integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
        * prediction_sampling_time_ / count_integral_A;
  }    
  model_B_ = integral_exp_A * B_continous_time;
  model_Bd_ = integral_exp_A * Bd_continous_time;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B_;
  //Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) = model_Bd_;

  std::cout << "Linear MPC: model matrix initialized correctly" << std::endl;
  printf("Linear MPC: A:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kStateSize; ++j){
      std::cout << model_A_(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLinear MPC: B:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kInputSize; ++j){
      std::cout << model_B_(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  printf("\nLinear MPC: Bd:\n");
  for(int i = 0; i < kStateSize; ++i){
    for(int j = 0; j < kDisturbanceSize; ++j){
      std::cout << model_Bd_(i,j) << "\t";
    }
    std::cout << std::endl;
  }
}
#endif


 //end of namespace Shared_control_mpc
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  using namespace shared_control_mpc;
  //LinearModelPredictiveControl linear_mpc_;
  set_defaults();  // Set basic algorithm parameters.
  setup_indexing();

  #if(!SOLVERTEST)
  load_data_model();
  load_data_penality();
  #endif
  load_data_x_ss();
  solve();
  #if(num_test > 0)
  settings.verbose = 1;
  tic();
  for (int i = 0; i < 20; ++i) {  // Main control loop.
    // Solve our problem at high speed!
    solve();
    linear_mpc_.use_solution();
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
  #endif
}