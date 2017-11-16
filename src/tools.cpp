#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() != 0 && estimations.size() == ground_truth.size())
  {
    VectorXd error(4);
    VectorXd sqError(4);
    for (int i=0; i<estimations.size(); ++i)
    {
      error = estimations[i] - ground_truth[i];
      sqError = error.array()*error.array();
      rmse += sqError;
    }
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
  }
  else
  {
    std::cout << "Invalid size for estimation vector!" << endl;
    return rmse;
  }
}
