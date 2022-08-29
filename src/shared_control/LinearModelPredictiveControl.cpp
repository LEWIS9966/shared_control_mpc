#include <px4_ros_com/LinearModelPredictiveControl.hpp>
#include <iostream>


using namespace shared_control_mpc;

void LinearModelPredictiveControl::load_data_x_ss(Params params){
  params.x_0[0] = 0.20319161029830202;
  params.x_0[1] = 0.8325912904724193;
  params.x_0[2] = -0.8363810443482227;
  params.x_0[3] = 0.04331042079065206;
  params.x_0[4] = 1.5717878173906188;
  params.x_0[5] = 1.5851723557337523;
  params.x_0[6] = -1.497658758144655;
  params.x_0[7] = -1.171028487447253;

  params.x_ss_0[0] = -1.7941311867966805;
  params.x_ss_0[1] = -0.23676062539745413;
  params.x_ss_0[2] = -1.8804951564857322;
  params.x_ss_0[3] = -0.17266710242115568;
  params.x_ss_0[4] = 0.596576190459043;
  params.x_ss_0[5] = -0.8860508694080989;
  params.x_ss_0[6] = 0.7050196079205251;
  params.x_ss_0[7] = 0.3634512696654033;
  params.u_ss_0[0] = 0.17859607212737894;
  params.u_ss_0[1] = 1.1212590580454682;
  params.u_ss_0[2] = -0.774545870495281;
  params.x_ss_1[0] = 0.6136436100941447;
  params.x_ss_1[1] = 0.2313630495538037;
  params.x_ss_1[2] = -0.5537409477496875;
  params.x_ss_1[3] = -1.0997819806406723;
  params.x_ss_1[4] = -0.3739203344950055;
  params.x_ss_1[5] = -0.12423900520332376;
  params.x_ss_1[6] = -0.923057686995755;
  params.x_ss_1[7] = -0.8328289030982696;
  params.u_ss_1[0] = -0.16925440270808823;
  params.u_ss_1[1] = 1.442135651787706;
  params.u_ss_1[2] = 0.34501161787128565;
  params.x_ss_2[0] = -0.8660485502711608;
  params.x_ss_2[1] = -0.8880899735055947;
  params.x_ss_2[2] = -0.1815116979122129;
  params.x_ss_2[3] = -1.17835862158005;
  params.x_ss_2[4] = -1.1944851558277074;
  params.x_ss_2[5] = 0.05614023926976763;
  params.x_ss_2[6] = -1.6510825248767813;
  params.x_ss_2[7] = -0.06565787059365391;
  params.u_ss_2[0] = -0.5512951504486665;
  params.u_ss_2[1] = 0.8307464872626844;
  params.u_ss_2[2] = 0.9869848924080182;
  params.x_ss_3[0] = 0.7643716874230573;
  params.x_ss_3[1] = 0.7567216550196565;
  params.x_ss_3[2] = -0.5055995034042868;
  params.x_ss_3[3] = 0.6725392189410702;
  params.x_ss_3[4] = -0.6406053441727284;
  params.x_ss_3[5] = 0.29117547947550015;
  params.x_ss_3[6] = -0.6967713677405021;
  params.x_ss_3[7] = -0.21941980294587182;
  params.u_ss_3[0] = -1.753884276680243;
  params.u_ss_3[1] = -1.0292983112626475;
  params.u_ss_3[2] = 1.8864104246942706;
  params.x_ss_4[0] = -1.077663182579704;
  params.x_ss_4[1] = 0.7659100437893209;
  params.x_ss_4[2] = 0.6019074328549583;
  params.x_ss_4[3] = 0.8957565577499285;
  params.x_ss_4[4] = -0.09964555746227477;
  params.x_ss_4[5] = 0.38665509840745127;
  params.x_ss_4[6] = -1.7321223042686946;
  params.x_ss_4[7] = -1.7097514487110663;
  params.u_ss_4[0] = -1.2040958948116867;
  params.u_ss_4[1] = -1.3925560119658358;
  params.u_ss_4[2] = -1.5995826216742213;
  params.x_ss_5[0] = -1.4828245415645833;
  params.x_ss_5[1] = 0.21311092723061398;
  params.x_ss_5[2] = -1.248740700304487;
  params.x_ss_5[3] = 1.808404972124833;
  params.x_ss_5[4] = 0.7264471152297065;
  params.x_ss_5[5] = 0.16407869343908477;
  params.x_ss_5[6] = 0.8287224032315907;
  params.x_ss_5[7] = -0.9444533161899464;
  params.u_ss_5[0] = 1.7069027370149112;
  params.u_ss_5[1] = 1.3567722311998827;
  params.u_ss_5[2] = 0.9052779937121489;
  params.x_ss_6[0] = -0.07904017565835986;
  params.x_ss_6[1] = 1.3684127435065871;
  params.x_ss_6[2] = 0.979009293697437;
  params.x_ss_6[3] = 0.6413036255984501;
  params.x_ss_6[4] = 1.6559010680237511;
  params.x_ss_6[5] = 0.5346622551502991;
  params.x_ss_6[6] = -0.5362376605895625;
  params.x_ss_6[7] = 0.2113782926017822;
  params.u_ss_6[0] = -1.2144776931994525;
  params.u_ss_6[1] = -1.2317108144255875;
  params.u_ss_6[2] = 0.9026784957312834;
  params.x_ss_7[0] = 1.1397468137245244;
  params.x_ss_7[1] = 1.8883934547350631;
  params.x_ss_7[2] = 1.4038856681660068;
  params.x_ss_7[3] = 0.17437730638329096;
  params.x_ss_7[4] = -1.6408365219077408;
  params.x_ss_7[5] = -0.04450702153554875;
  params.x_ss_7[6] = 1.7117453902485025;
  params.x_ss_7[7] = 1.1504727980139053;
  params.u_ss_7[0] = -0.05962309578364744;
  params.u_ss_7[1] = -0.1788825540764547;
  params.u_ss_7[2] = -1.1280569263625857;
  params.x_ss_8[0] = -1.2911464767927057;
  params.x_ss_8[1] = -1.7055053231225696;
  params.x_ss_8[2] = 1.56957275034837;
  params.x_ss_8[3] = 0.5607064675962357;
  params.x_ss_8[4] = -1.4266707301147146;
  params.x_ss_8[5] = -0.3434923211351708;
  params.x_ss_8[6] = -1.8035643024085055;
  params.x_ss_8[7] = -1.1625066019105454;
  params.u_ss_8[0] = 0.9228324965161532;
  params.u_ss_8[1] = 0.6044910817663975;
  params.u_ss_8[2] = -0.0840868104920891;
  params.x_ss_9[0] = -0.900877978017443;
  params.x_ss_9[1] = 0.608892500264739;
  params.x_ss_9[2] = 1.8257980452695217;
  params.x_ss_9[3] = -0.25791777529922877;
  params.x_ss_9[4] = -1.7194699796493191;
  params.x_ss_9[5] = -1.7690740487081298;
  params.x_ss_9[6] = -1.6685159248097703;
  params.x_ss_9[7] = 1.8388287490128845;
  params.u_ss_9[0] = 0.16304334474597537;
  params.u_ss_9[1] = 1.3498497306788897;
  params.u_ss_9[2] = -1.3198658230514613;
  params.x_ss_10[0] = -0.9586197090843394;
  params.x_ss_10[1] = 0.7679100474913709;
  params.x_ss_10[2] = 1.5822813125679343;
  params.x_ss_10[3] = -0.6372460621593619;
  params.x_ss_10[4] = -1.741307208038867;
  params.x_ss_10[5] = 1.456478677642575;
  params.x_ss_10[6] = -0.8365102166820959;
  params.x_ss_10[7] = 0.9643296255982503;
  params.u_ss_10[0] = -1.367865381194024;
  params.u_ss_10[1] = 0.7798537405635035;
  params.u_ss_10[2] = 1.3656784761245926;
  params.x_ss_11[0] = 0.9086083149868371;
  params.x_ss_11[1] = -0.5635699005460344;
  params.x_ss_11[2] = 0.9067590059607915;
  params.x_ss_11[3] = -1.4421315032701587;
  params.x_ss_11[4] = -0.7447235390671119;
  params.x_ss_11[5] = -0.32166897326822186;
  params.x_ss_11[6] = 1.5088481557772684;
  params.x_ss_11[7] = -1.385039165715428;
  params.u_ss_11[0] = 1.5204991609972622;
  params.u_ss_11[1] = 1.1958572768832156;
  params.u_ss_11[2] = 1.8864971883119228;
  params.x_ss_12[0] = -0.5291880667861584;
  params.x_ss_12[1] = -1.1802409243688836;
  params.x_ss_12[2] = -1.037718718661604;
  params.x_ss_12[3] = 1.3114512056856835;
  params.x_ss_12[4] = 1.8609125943756615;
  params.x_ss_12[5] = 0.7952399935216938;
  params.x_ss_12[6] = -0.07001183290468038;
  params.x_ss_12[7] = -0.8518009412754686;
  params.u_ss_12[0] = 1.3347515373726386;
  params.u_ss_12[1] = 1.4887180335977037;
  params.u_ss_12[2] = -1.6314736327976336;
  params.x_ss_13[0] = -1.1362021159208933;
  params.x_ss_13[1] = 1.327044361831466;
  params.x_ss_13[2] = 1.3932155883179842;
  params.x_ss_13[3] = -0.7413880049440107;
  params.x_ss_13[4] = -0.8828216126125747;
  params.x_ss_13[5] = -0.27673991192616;
  params.x_ss_13[6] = 0.15778600105866714;
  params.x_ss_13[7] = -1.6177327399735457;
  params.u_ss_13[0] = 1.3476485548544606;
  params.u_ss_13[1] = 0.13893948140528378;
  params.u_ss_13[2] = 1.0998712601636944;
  params.x_ss_14[0] = -1.0766549376946926;
  params.x_ss_14[1] = 1.8611734044254629;
  params.x_ss_14[2] = 1.0041092292735172;
  params.x_ss_14[3] = -0.6276245424321543;
  params.x_ss_14[4] = 1.794110587839819;
  params.x_ss_14[5] = 0.8020471158650913;
  params.x_ss_14[6] = 1.362244341944948;
  params.x_ss_14[7] = -1.8180107765765245;
  params.u_ss_14[0] = -1.7774338357932473;
  params.u_ss_14[1] = 0.9709490941985153;
  params.u_ss_14[2] = -0.7812542682064318;
  params.x_ss_15[0] = 0.0671374633729811;
  params.x_ss_15[1] = -1.374950305314906;
  params.x_ss_15[2] = 1.9118096386279388;
  params.x_ss_15[3] = 0.011004190697677885;
  params.x_ss_15[4] = 1.3160043138989015;
  params.x_ss_15[5] = -1.7038488148800144;
  params.x_ss_15[6] = -0.08433819112864738;
  params.x_ss_15[7] = -1.7508820783768964;
  params.u_ss_15[0] = 1.536965724350949;
  params.u_ss_15[1] = -0.21675928514816478;
  params.u_ss_15[2] = -1.725800326952653;
  params.x_ss_16[0] = -1.6940148707361717;
  params.x_ss_16[1] = 0.15517063201268;
  params.x_ss_16[2] = -1.697734381979077;
  params.x_ss_16[3] = -1.264910727950229;
  params.x_ss_16[4] = -0.2545716633339441;
  params.x_ss_16[5] = -0.008868675926170244;
  params.x_ss_16[6] = 0.3332476609670296;
  params.x_ss_16[7] = 0.48205072561962936;
  params.u_ss_16[0] = -0.5087540014293261;
  params.u_ss_16[1] = 0.4749463319223195;
  params.u_ss_16[2] = -1.371021366459455;
  params.x_ss_17[0] = -0.8979660982652256;
  params.x_ss_17[1] = 1.194873082385242;
  params.x_ss_17[2] = -1.3876427970939353;
  params.x_ss_17[3] = -1.106708108457053;
  params.x_ss_17[4] = -1.0280872812241797;
  params.x_ss_17[5] = -0.08197078070773234;
  params.x_ss_17[6] = -1.9970179118324083;
  params.x_ss_17[7] = -1.878754557910134;
  params.u_ss_17[0] = -0.15380739340877803;
  params.u_ss_17[1] = -1.349917260533923;
  params.u_ss_17[2] = 0.7180072150931407;
  params.x_ss_18[0] = 1.1808183487065538;
  params.x_ss_18[1] = 0.31265343495084075;
  params.x_ss_18[2] = 0.7790599086928229;
  params.x_ss_18[3] = -0.4361679370644853;
  params.x_ss_18[4] = -1.8148151880282066;
  params.x_ss_18[5] = -0.24231386948140266;
  params.x_ss_18[6] = -0.5120787511622411;
  params.x_ss_18[7] = 0.3880129688013203;
  params.u_ss_18[0] = -1.4631273212038676;
  params.u_ss_18[1] = -1.0891484131126563;
  params.u_ss_18[2] = 1.2591296661091191;
  params.x_ss_19[0] = -0.9426978934391474;
  params.x_ss_19[1] = -0.358719180371347;
  params.x_ss_19[2] = 1.7438887059831263;
  params.x_ss_19[3] = -0.8977901479165817;
  params.x_ss_19[4] = -1.4188401645857445;
  params.x_ss_19[5] = 0.8080805173258092;
  params.x_ss_19[6] = 0.2682662017650985;
  params.x_ss_19[7] = 0.44637534218638786;
  params.d[0] = -1.214148157218884;
  params.d[1] = 1.6818776980374155;
  params.d[2] = -0.30341101038214635;
  params.u_prev[0] = 1.9039816898917352;
  params.u_prev[1] = 0.6895347036512547;
  params.u_prev[2] = 1.6113364341535923;
}

void LinearModelPredictiveControl::load_data_penality(Params params){
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
}
void LinearModelPredictiveControl::load_data_model(Params params) {
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
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) = model_Bd_;

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

void LinearModelPredictiveControl::use_solution(Vars vars) {
  // In this function, use the optimization result.
  printf("\troll: %f\tpitch: %f\tthrust:%f\n",vars.u_0[0],vars.u_0[1],vars.u_0[2]);
}
 //end of namespace Shared_control_mpc
int main(int argc, char **argv) {
  using namespace shared_control_mpc;
  LinearModelPredictiveControl linear_mpc_;
  set_defaults();  // Set basic algorithm parameters.
  setup_indexing();
  linear_mpc_.load_data_model(params);
  linear_mpc_.load_data_penality(params);
  linear_mpc_.load_data_x_ss(params);
  int num_iters_ ,num_test = 0;
  num_iters_ = solve();
  linear_mpc_.use_solution(vars);
  #if (num_test > 0)
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
  #endif
}