#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
  public:
  struct Output_t {
    vector<double> x;
    vector<double> y;
    vector<double> psi;
    vector<double> v;
    vector<double> cte;
    vector<double> epsi;
    vector<double> delta;
    vector<double> a;
  };

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Output_t Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
