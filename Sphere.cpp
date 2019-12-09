#include <stdio.h>
#include <iostream>

#include <Eigen/Sparse>


#include <ceres/ceres.h>

#include "Sphere.h"

class PoseGraphError : public ceres::SizedCostFunction<6, 6, 6> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseGraphError(int i, Sophus::SE3d &T_ij, Eigen::Matrix<double, 6, 6> &information) {
        T_ij_ = T_ij;
        Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(information);
        sqrt_information_ = llt.matrixL();
        id = i;
    }


    // T_i 是 T_wi, 是 i 在世界坐标系中的坐标，与 orb 中的位姿相反，orb 中 T_i 是
    // 世界坐标系在 i 中的坐标，即 T_iw
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie_i(*parameters);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie_j(*(parameters + 1));

        Sophus::SE3d T_i = Sophus::SE3d::exp(lie_i);
        Sophus::SE3d T_j = Sophus::SE3d::exp(lie_j);
        Sophus::SE3d Tij_estimate = T_i.inverse() * T_j;
        Sophus::SE3d err = Tij_estimate * T_ij_.inverse();
        Eigen::Map<Eigen::Matrix<double, 6, 1>> err_(residuals);
        err_ = err.log();

        Eigen::Matrix<double, 6, 6> Jac_i;
        Eigen::Matrix<double, 6, 6> Jac_j;
        Eigen::Matrix<double, 6, 6> Jl;
        Jl.block(0, 0, 3, 3) = Sophus::SO3d::hat(err_.tail(3));
        Jl.block(3, 3, 3, 3) = Jl.block(0, 0, 3, 3);
        Jl.block(0, 3, 3, 3) = Sophus::SO3d::hat(err_.head(3));
        Jl.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
        Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        Jl.noalias() = sqrt_information_ * (I - 0.5 * Jl);// + 1.0/12. * (Jl * Jl));

        err_ = sqrt_information_ * err_;

        Jac_i = -Jl * T_i.inverse().Adj();
        Jac_j = -Jac_i;
        int k = 0;
        for(int i = 0; i < 6; i++) {
            if(jacobians) {
                for (int j = 0; j < 6; ++j) {
                    if (jacobians[0])
                        jacobians[0][k] = Jac_i(i, j);
                    if (jacobians[1])
                        jacobians[1][k] = Jac_j(i, j);
                    k++;
                }
            }
        }
        return true;
    }

private:
    int id;
    Sophus::SE3d T_ij_;
    Eigen::Matrix<double, 6, 6> sqrt_information_;
};




Sphere::Sphere()
{
}


class CERES_EXPORT SE3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~SE3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};

bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

bool SE3Parameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);
    Sophus::SE3d T = Sophus::SE3d::exp(lie);
    Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);
    Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();
    for(int i = 0; i < 6; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);
    return true;
}


bool Sphere::optimize(int iter_) {
    if(vertexes.empty() == true || edges.empty() == true)
        return false;

    ceres::Problem problem;
    for(size_t i = 0; i < edges.size(); ++i) {
        ceres::CostFunction* costFun = new PoseGraphError(i, edges[i].pose, edges[i].infomation);
        problem.AddResidualBlock(costFun, new ceres::HuberLoss(1.5), vertexes[edges[i].i].pose.data(),
                                 vertexes[edges[i].j].pose.data());
    }

    for(size_t i = 0; i < vertexes.size(); ++i) {
        problem.SetParameterization(vertexes[i].pose.data(), new SE3Parameterization());
    }

    problem.SetParameterBlockConstant(vertexes[0].pose.data());


    //printf("optimization start!\n");

    ceres::Solver::Options options;
    options.max_num_iterations = iter_;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    // options.dynamic_sparsity = true;
    // options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    // options.minimizer_type = ceres::TRUST_REGION;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    return true;
}



