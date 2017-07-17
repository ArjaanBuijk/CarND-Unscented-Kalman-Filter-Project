#include <iostream>
#include "test.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Test::Test() {}

Test::~Test() {}

bool Test::test_weight_initialization(const VectorXd& weights) const{

  if (VERBOSE_){
    std::cout<<"-----------------------------------------------------------------\n";
    std::cout<<"Function test_weight_initialization";
    std::cout<<"weights =\n"<<weights<<'\n';

  }

  VectorXd weights_correct(15);
  weights_correct << -1.3333333,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667;

  for (int i=0; i<weights.rows(); ++i){
    double diff = weights(i)-weights_correct(i);
    if (fabs(diff)>TOLERANCE_){
        cout<<"ERROR: Initial Weight incorrect at index "<<i<<'\n';
        cout<<weights(i)<<" vs. "<<weights_correct(i)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
    }
  }

  return true;
}


