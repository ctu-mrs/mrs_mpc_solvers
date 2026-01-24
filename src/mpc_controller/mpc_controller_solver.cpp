/* author: Daniel Hert */

#include <mrs_mpc_solvers/mpc_controller.h>

using namespace Eigen;

namespace mrs_mpc_solvers
{

namespace mpc_controller
{

/* class Solver() //{ */

Solver::Solver(std::string name, bool verbose, int max_iters, std::vector<double> Q, std::vector<double> Q_last, double dt1, double dt2, double p1, double p2) {

  memset(&qp_solver_, 0, sizeof(QPSolver));

  this->_name_ = name;

  this->Q_          = Q;
  this->Q_last_     = Q_last;
  this->_verbose_   = verbose;
  this->_max_iters_ = max_iters;
  this->dt1_        = dt1;
  this->dt2_        = dt2;
  this->p1_         = p1;
  this->p2_         = p2;

  qp_solver_.paramsController.u_last[0] = 0;

  qp_solver_.set_defaults_controller();
  qp_solver_.setup_indexing_controller();
  qp_solver_.setup_indexed_optvarsController_controller();

  setParams();
}

//}

/* setParams() //{ */

void Solver::setParams(void) {

  qp_solver_.settingsController.verbose   = this->_verbose_;
  qp_solver_.settingsController.max_iters = this->_max_iters_;

  for (int i = 0; i < 3; i++) {
    qp_solver_.paramsController.Q[i] = Q_[i];
  }

  for (int i = 0; i < 3; i++) {
    qp_solver_.paramsController.Q_last[i] = Q_last_[i];
  }

  qp_solver_.paramsController.Af[0] = 1;
  qp_solver_.paramsController.Af[1] = 1;
  qp_solver_.paramsController.Af[2] = p1_;
  qp_solver_.paramsController.Af[3] = dt1_;
  qp_solver_.paramsController.Af[4] = dt1_;

  qp_solver_.paramsController.Bf[0] = p2_;

  qp_solver_.paramsController.A[0] = 1;
  qp_solver_.paramsController.A[1] = 1;
  qp_solver_.paramsController.A[2] = p1_;
  qp_solver_.paramsController.A[3] = dt2_;
  qp_solver_.paramsController.A[4] = dt2_;

  qp_solver_.paramsController.B[0] = p2_;
}

//}

/* setLimits() //{ */

void Solver::setLimits(double max_speed, double max_acc, double max_u, double max_du, double dt1, double dt2) {

  qp_solver_.paramsController.x_max_2[0]  = max_speed;
  qp_solver_.paramsController.x_max_3[0]  = max_acc;
  qp_solver_.paramsController.u_max[0]    = max_u;
  qp_solver_.paramsController.du_max_f[0] = max_du * dt1;
  qp_solver_.paramsController.du_max[0]   = max_du * dt2;
}

//}

/* setLastInput() //{ */

void Solver::setLastInput(double last_input) {

  qp_solver_.paramsController.u_last[0] = last_input;
}

//}

/* setDt() //{ */

void Solver::setDt(double dt1, double dt2) {

  qp_solver_.paramsController.Af[2] = dt1;
  qp_solver_.paramsController.Af[3] = dt1;

  qp_solver_.paramsController.A[2] = dt2;
  qp_solver_.paramsController.A[3] = dt2;
}

//}

/* setInitialState() //{ */

void Solver::setInitialState(MatrixXd& x) {

  qp_solver_.paramsController.x_0[0] = x(0, 0);
  qp_solver_.paramsController.x_0[1] = x(1, 0);
  qp_solver_.paramsController.x_0[2] = x(2, 0);
}

//}

/* loadReference() //{ */

void Solver::loadReference(MatrixXd& reference) {

  for (int i = 0; i < _horizon_len_; i++) {

    qp_solver_.paramsController.x_ss[i + 1][0] = reference((3 * i) + 0, 0);
    qp_solver_.paramsController.x_ss[i + 1][1] = reference((3 * i) + 1, 0);
    qp_solver_.paramsController.x_ss[i + 1][2] = reference((3 * i) + 2, 0);
  }
}

//}

/* setQ() //{ */

void Solver::setQ(const std::vector<double> new_Q) {

  for (int i = 0; i < _horizon_len_; i++) {

    this->Q_ = new_Q;
  }
}

//}

/* setS() //{ */

void Solver::setS(const std::vector<double> new_S) {

  for (int i = 0; i < _horizon_len_; i++) {

    this->Q_last_ = new_S;
  }
}

//}

/* solve() //{ */

int Solver::solveMPC() {

  return qp_solver_.solve_controller();
}

//}

/* getStates() //{ */

void Solver::getStates(MatrixXd& future_traj) {

  for (int i = 0; i < _horizon_len_; i++) {

    future_traj(0 + (i * 3)) = *(qp_solver_.varsController.x[i + 1]);
    future_traj(1 + (i * 3)) = *(qp_solver_.varsController.x[i + 1] + 1);
    future_traj(2 + (i * 3)) = *(qp_solver_.varsController.x[i + 1] + 2);
  }
}

//}

/* getFirstControlInput() //{ */

double Solver::getFirstControlInput() {

  return *(qp_solver_.varsController.u_0);
}

//}

}  // namespace mpc_controller

}  // namespace mrs_mpc_solvers
