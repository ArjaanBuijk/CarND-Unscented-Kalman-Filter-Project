#ifndef TEST_H_
#define TEST_H_
#include <vector>
#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Test {
public:
  /**
  * Constructor.
  */
  Test();

  /**
  * Destructor.
  */
  virtual ~Test();

  double TOLERANCE_ = 1.E-6;
  bool VERBOSE_ = true;

  /** A helper method to test weight initialization. */
  bool test_weight_initialization(const VectorXd& weights) const;

};

#endif /* TEST_H_ */
