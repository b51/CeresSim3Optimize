#include <iostream>
#include <stdio.h>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <ceres/ceres.h>
#include <sophus/sim3.hpp>

#include "Sim3Optimizer.h"

class PoseGraphError : public ceres::SizedCostFunction<7, 7, 7> {
public:
  PoseGraphError(const Sophus::Sim3d &Sji,
                 const Eigen::Matrix<double, 7, 7> &information)
      : Sji_(Sji) {
    Eigen::LLT<Eigen::Matrix<double, 7, 7>> llt(information);
    sqrt_information_ = llt.matrixL();
  }

  // Si means world frame in i frame, S_iw
  virtual bool Evaluate(double const *const *parameters_ptr,
                        double *residuals_ptr, double **jacobians_ptr) const {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_j(*parameters_ptr);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_i(*(parameters_ptr + 1));

    Sophus::Sim3d Si = Sophus::Sim3d::exp(lie_i);
    Sophus::Sim3d Sj = Sophus::Sim3d::exp(lie_j);
    Sophus::Sim3d error = Sji_ * Si * Sj.inverse();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> residuals(residuals_ptr);
    residuals = error.log();

    if (jacobians_ptr) {
      Eigen::Matrix<double, 7, 7> Jacobian_i;
      Eigen::Matrix<double, 7, 7> Jacobian_j;
      Eigen::Matrix<double, 7, 7> Jr = Eigen::Matrix<double, 7, 7>::Zero();

      Jr.block<3, 3>(0, 0) = Sophus::RxSO3d::hat(residuals.tail(4));
      Jr.block<3, 3>(0, 3) = Sophus::SO3d::hat(residuals.head(3));
      Jr.block<3, 1>(0, 6) = -residuals.head(3);
      Jr.block<3, 3>(3, 3) = Sophus::SO3d::hat(residuals.block<3, 1>(3, 0));
      Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
      Jr = sqrt_information_ * (I + 0.5 * Jr + 1.0 / 12. * (Jr * Jr));

      Jacobian_i = Jr * Sj.Adj();
      Jacobian_j = -Jacobian_i;
      int k = 0;
      for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; ++j) {
          if (jacobians_ptr[0])
            jacobians_ptr[0][k] = Jacobian_j(i, j);
          if (jacobians_ptr[1])
            jacobians_ptr[1][k] = Jacobian_i(i, j);
          k++;
        }
      }
    }
    residuals = sqrt_information_ * residuals;
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction *
  Create(const Sophus::Sim3d &Sji,
         const Eigen::Matrix<double, 7, 7> &sqrt_information) {
    return new PoseGraphError(Sji, sqrt_information);
  }

private:
  const Sophus::Sim3d Sji_;
  Eigen::Matrix<double, 7, 7> sqrt_information_;
};

Sim3Optimizer::Sim3Optimizer() {}

class CERES_EXPORT Sim3Parameterization : public ceres::LocalParameterization {
public:
  virtual ~Sim3Parameterization() {}
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 7; }
};

bool Sim3Parameterization::Plus(const double *x, const double *delta,
                                double *x_plus_delta) const {
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie(x);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta_lie(delta);
  Sophus::Sim3d T = Sophus::Sim3d::exp(lie);
  Sophus::Sim3d delta_T = Sophus::Sim3d::exp(delta_lie);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> x_plus_delta_lie(x_plus_delta);
  x_plus_delta_lie = (T * delta_T).log();
  return true;
}

bool Sim3Parameterization::ComputeJacobian(const double *x,
                                           double *jacobian) const {
  ceres::MatrixRef(jacobian, 7, 7) = ceres::Matrix::Identity(7, 7);
  return true;
}

bool Sim3Optimizer::optimize(int iter) {
  if (vertexes.empty() == true || edges.empty() == true)
    return false;

  ceres::Problem problem;
  for (size_t m = 0; m < edges.size(); ++m) {
    ceres::CostFunction *cost_function =
        PoseGraphError::Create(edges[m].pose, edges[m].information);
    problem.AddResidualBlock(cost_function, nullptr,
                             vertexes[edges[m].j].data(),
                             vertexes[edges[m].i].data());
  }

  for (auto &it : vertexes) {
    problem.SetParameterization(it.second.data(), new Sim3Parameterization());
  }

  problem.SetParameterBlockConstant(vertexes[70].data());

  ceres::Solver::Options options;
  options.max_num_iterations = iter;
  options.minimizer_progress_to_stdout = true;
  options.function_tolerance = 1e-16;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  return true;
}

void Sim3Optimizer::ErrorAndJacobianCalculation(
    const Sophus::Sim3d& Sji, const Eigen::Matrix<double, 7, 7>& information,
    const Eigen::Matrix<double, 7, 1>& lie_i,
    const Eigen::Matrix<double, 7, 1>& lie_j,
    Eigen::Matrix<double, 7, 1>& residuals,
    Eigen::Matrix<double, 7, 7>& Jacobian_i,
    Eigen::Matrix<double, 7, 7>& Jacobian_j) {
  const Sophus::Sim3d _Sji = Sji;
  Sophus::Sim3d Si = Sophus::Sim3d::exp(lie_i);
  Sophus::Sim3d Sj = Sophus::Sim3d::exp(lie_j);
  Sophus::Sim3d Serror = _Sji * (Si * Sj.inverse());

  Eigen::Matrix<double, 7, 1> error = Serror.log();
  Eigen::Matrix<double, 7, 7> Jr = Eigen::Matrix<double, 7, 7>::Zero();

  Jr.block<3, 3>(0, 0) = Sophus::RxSO3d::hat(error.tail(4));
  Jr.block<3, 3>(0, 3) = Sophus::SO3d::hat(error.head(3));
  Jr.block<3, 1>(0, 6) = -error.head(3);
  Jr.block<3, 3>(3, 3) = Sophus::SO3d::hat(error.block<3, 1>(3, 0));
  Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
  Jr = information * (I + 0.5 * Jr + 1.0 / 12. * (Jr * Jr));

  Jacobian_i = Jr * Sj.Adj();
  Jacobian_j = -Jacobian_i;
  residuals = information * error;
}

double Sim3Optimizer::IterateOnce(
    Eigen::Matrix<double, Eigen::Dynamic, 1>& delta_sim) {
  int n_error = (int)edges.size();
  int n_vertex = (int)vertexes.size();
  delta_sim.resize(n_vertex * 7, 1);

  Eigen::MatrixXd Jacobian(n_error * 7, n_vertex * 7);
  Eigen::MatrixXd error(n_error * 7, 1);

  for (size_t m = 0; m < edges.size(); m++) {
    Eigen::Matrix<double, 7, 7> Jacobian_i, Jacobian_j;
    Eigen::Matrix<double, 7, 1> residuals;

    int i = vertexes_remapped[edges[m].i];
    int j = vertexes_remapped[edges[m].j];

    ErrorAndJacobianCalculation(edges[m].pose, edges[m].information,
                                vertexes[edges[m].i], vertexes[edges[m].j],
                                residuals, Jacobian_i, Jacobian_j);
    Jacobian.block<7, 7>(m * 7, i * 7) = Jacobian_i;
    Jacobian.block<7, 7>(m * 7, j * 7) = Jacobian_j;
    error.block<7, 1>(m * 7, 0) = residuals;
  }

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
  Eigen::Matrix<double, Eigen::Dynamic, 1> b;
  A.resize(n_vertex * 7, n_vertex * 7);
  b.resize(n_vertex * 7, 1);

  A = Jacobian.transpose() * Jacobian;
  b = -Jacobian.transpose() * error;

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;//(A);
  ldlt.compute(A.sparseView());
  delta_sim = ldlt.solve(b);
  return error.norm();
}

/**
 *        S0
 *        S1
 *        ..
 *  X = [ Si ]
 *        ..
 *        Sn
 *              ..., d(error0) / d(Si), ..., d(error0)/d(Sj), ... 
 *              ..., d(error1) / d(Si), ..., d(error1)/d(Sj), ... 
 *                                      ...
 *  Jacobian = [..., d(errorm) / d(Si), ..., d(errorm)/d(Sj), ... ]
 *                                      ...
 *              ..., d(errorn) / d(Si), ..., d(errorn)/d(Sj), ... 
 *
 */
bool Sim3Optimizer::LocalBAOptimize(int iter) {
  int index = 0;
  double last_error = -1.0;
  for (auto it = vertexes.begin(); it != vertexes.end(); it++) {
    vertexes_remapped[it->first] = index;
    inversed_vertexes_remapped[index] = it->first;
    index++;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_sim;
  for (int n = 0; n < iter; n++) {
    double error = IterateOnce(delta_sim);
    if (error > 0 && error < 1e-6) {
      LOG(INFO) << "Iteration times: " << n << " error: " << error;
      break;
    }
    if (std::abs(error - last_error) < 1e-6) {
      LOG(INFO) << "Iteration times: " << n << " error: " << error;
      break;
    }
    last_error = error;
    for (size_t i = 0; i < vertexes.size(); i++) {
      Eigen::Matrix<double, 7, 1> lie_i =
          vertexes[inversed_vertexes_remapped[i]];
      Eigen::Matrix<double, 7, 1> delta_i = delta_sim.block<7, 1>(i * 7, 0);
      Sophus::Sim3d updated =
          Sophus::Sim3d::exp(lie_i) * Sophus::Sim3d::exp(delta_i);
      vertexes[inversed_vertexes_remapped[i]] = updated.log();
    }
  }
  return true;
}
