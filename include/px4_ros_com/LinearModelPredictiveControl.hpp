#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <solver.h>

Vars vars;
Params params;
Workspace work;
Settings settings;

namespace shared_control_mpc{

static constexpr int kStateSize = 8;
static constexpr int kInputSize = 3;
static constexpr int kMeasurementSize = 6;
static constexpr int kDisturbanceSize = 3;
static constexpr int kPredictionHorizonSteps = 20;
static constexpr double kGravity = 9.8066;
static constexpr double roll_time_constant_ = 0.25;
static constexpr double pitch_time_constant_ = 0.255;
static constexpr double prediction_sampling_time_ = 0.1;
static constexpr double sampling_time_ = 0.01;
static constexpr double roll_gain_ = 0.9;
static constexpr double pitch_gain_ = 0.9;
static constexpr double mass_ = 1.52;
static constexpr double roll_limit_ = 0.436332;
static constexpr double pitch_limit_ = 0.436332;
static constexpr int thrust_max_ = 20;
static constexpr int thrust_min_ = 5;

class LinearModelPredictiveControl
{
private:
    
public:
    Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
    Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
    Eigen::Matrix<double, kStateSize, kInputSize> model_Bd_;
    LinearModelPredictiveControl(/* args */);
    ~LinearModelPredictiveControl();
    void load_data_x_ss(Params params);
    void load_data_penality(Params params);
    void load_data_model(Params params);
    void use_solution(Vars vars);
};

LinearModelPredictiveControl::LinearModelPredictiveControl(/* args */)
{
}

LinearModelPredictiveControl::~LinearModelPredictiveControl()
{
}
}
