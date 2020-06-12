#include <eigen3/Eigen/Eigen>
#include <mpc_tracker_solver.h>

using namespace Eigen;

namespace mrs_mpc_solvers
{

namespace mpc_tracker
{

extern "C" {
#include "cvxgen/solver.h"
}

Vars      vars;
Params    params;
Workspace work;
Settings  settings;

/* class Solver() //{ */

/* _dim_ is used to offset the result in the output vecor, according to which dimension (x,y,z) is being calculated */
/* x - 0 */
/* y - 1 */
/* z - 2 */
/* yaw - 0 */

Solver::Solver(std::string name, bool verbose, int max_iters, std::vector<double> tempQ, double dt, double dt2, int dimension) {

  _name_ = name;

  myQ_ = std::vector<double>(4);
  set_defaults();
  setup_indexing();
  setup_indexed_params();
  _dim_ = dimension * 4;

  if (_dim_ > 8 || _dim_ < 0) {
    ROS_ERROR("[%s]: solver - parameter _dim_ should be 0, 1 or 2 !!! setting to 0", _name_.c_str());
    _dim_ = 0;
  }

  if (verbose) {
    settings.verbose = 1;
  } else {
    settings.verbose = 0;
  }

  if ((max_iters < 1 || max_iters > 100) || !std::isfinite(max_iters)) {
    ROS_ERROR("[%s]: solver - max_iters wrong value!!! Safe value of 20 set instead", _name_.c_str());
    max_iters = 20;
  }
  settings.max_iters = max_iters;

  if (tempQ.size() == 4) {
    for (int i = 0; i < 4; i++) {
      if (tempQ[i] >= 0 && std::isfinite(tempQ[i])) {
        myQ_[i] = tempQ[i];
      } else {
        ROS_ERROR("[%s]: solver - Q matrix has to be PSD - parameter %d !!! Safe value of 500 set instead", _name_.c_str(), i);
        myQ_[i] = 500;
      }
    }
  } else {

    ROS_ERROR("[%s]: solver - Q matrix wrong size %d !!! Safe values set instead", _name_.c_str(), int(tempQ.size()));

    myQ_[0] = 5000;
    myQ_[1] = 0;
    myQ_[2] = 0;
    myQ_[3] = 0;
  }

  if (dt <= 0 || !std::isfinite(dt)) {
    ROS_ERROR("[%s]: solver - dt parameter wrong %.3f !!! Safe value of 0.01 set instead", _name_.c_str(), dt);
    dt = 0.01;
  }

  if (dt2 <= 0 || !std::isfinite(dt2)) {
    ROS_ERROR("[%s]: solver - dt2 parameter wrong %.3f !!! Safe value of 0.2 set instead", _name_.c_str(), dt2);
    dt2 = 0.2;
  }

  params.A[0] = 1;
  params.A[1] = 1;
  params.A[2] = 1;
  params.A[3] = 1;
  params.A[4] = dt2;
  params.A[5] = dt2;
  params.A[6] = dt2;
  params.A[7] = 0.5 * dt2 * dt2;
  params.A[8] = 0.5 * dt2 * dt2;

  params.B[0] = dt2;

  params.Af[0] = 1;
  params.Af[1] = 1;
  params.Af[2] = 1;
  params.Af[3] = 1;
  params.Af[4] = dt;
  params.Af[5] = dt;
  params.Af[6] = dt;
  params.Af[7] = 0.5 * dt * dt;
  params.Af[8] = 0.5 * dt * dt;

  params.Bf[0] = dt;

  ROS_INFO("[%s]: solver initialized", _name_.c_str());
}

//}

/* setLimits() //{ */

void Solver::setLimits(double max_speed, double min_speed, double max_acc, double min_acc, double max_jerk, double min_jerk, double max_snap, double min_snap) {

  params.x_max_2[0] = max_speed;
  params.x_min_2[0] = min_speed;
  params.x_max_3[0] = max_acc;
  params.x_min_3[0] = min_acc;
  params.x_max_4[0] = max_jerk;
  params.x_min_4[0] = min_jerk;
  params.u_max[0]   = max_snap;
  params.u_min[0]   = min_snap;
}

//}

/* setInitialState() //{ */

void Solver::setInitialState(MatrixXd& x) {

  params.x_0[0] = x(0, 0);
  params.x_0[1] = x(1, 0);
  params.x_0[2] = x(2, 0);
  params.x_0[3] = x(3, 0);
}

//}

/* setVelQ() //{ */

bool Solver::setVelQ(double Q_vel) {

  if (Q_vel < 0.0) {

    ROS_ERROR("[%s]: solver - Q vel has to be positive!! Q_vel = %.2f !!!", _name_.c_str(), Q_vel);
    return false;

  } else {

    myQ_[1] = Q_vel;
    return true;
  }
}

//}

/* setQ() //{ */

bool Solver::setQ(std::vector<double> Qnew) {

  bool result = true;

  if (Qnew.size() == 4) {

    for (int i = 0; i < 4; i++) {

      if (Qnew[i] >= 0 && std::isfinite(Qnew[i])) {
        myQ_[i] = Qnew[i];
      } else {
        ROS_ERROR("[%s]: solver - Q matrix has to be PSD - parameter #n %d !!!", _name_.c_str(), i);
        result = false;
      }
    }

  } else {
    ROS_ERROR("[%s]: solver - wrong dimension received when setting Q", _name_.c_str());
    result = false;
  }
  if (result) {
    ROS_INFO("[%s]: solver - successfully set matrix Q", _name_.c_str());
  }
  return result;
}

//}

/* loadReference() //{ */

void Solver::loadReference(MatrixXd& reference) {

  for (int i = 0; i < _horizon_len_; i++) {
    *params.x_ss[i + 1] = reference(i, 0);
  }
}

//}

/* solve() //{ */

int Solver::solveMPC() {

  for (int i = 0; i < 4; i++) {
    params.Q[i] = myQ_[i];
  }

  return solve();
}

//}

/* getStates() //{ */

void Solver::getStates(MatrixXd& future_traj) {

  for (int i = 0; i < _horizon_len_; i++) {
    future_traj(0 + _dim_ + (i * 12)) = *(vars.x[i + 1]);
    future_traj(1 + _dim_ + (i * 12)) = *(vars.x[i + 1] + 1);
    future_traj(2 + _dim_ + (i * 12)) = *(vars.x[i + 1] + 2);
    future_traj(3 + _dim_ + (i * 12)) = *(vars.x[i + 1] + 3);
  }
}

//}

/* getFirstControlInput() //{ */

double Solver::getFirstControlInput() {

  return *(vars.u_0);
}

//}

}  // namespace mpc_tracker

}  // namespace mrs_mpc_solvers
