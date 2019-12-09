#include <stdio.h>
#include <iostream>

#include <Eigen/Sparse>


#include <ceres/ceres.h>

#include "Sphere.h"

class PoseGraphError : public ceres::SizedCostFunction<7, 7, 7> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseGraphError(int i, Sophus::Sim3d &Tji, Eigen::Matrix<double, 7, 7> &information) {
        Tji_ = Tji;
        // Eigen::LLT<Eigen::Matrix<double, 7, 7>> llt(information);
        sqrt_information_ = information;
        id = i;
    }


    // T_i 是 T_wi, 是 i 在世界坐标系中的坐标，与 orb 中的位姿相反，orb 中 T_i 是
    // 世界坐标系在 i 中的坐标，即 T_iw
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_j(*parameters);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_i(*(parameters + 1));

        Sophus::Sim3d T_i = Sophus::Sim3d::exp(lie_i);
        Sophus::Sim3d T_j = Sophus::Sim3d::exp(lie_j);
        Sophus::Sim3d Tij_estimate = T_i * T_j.inverse();
        Sophus::Sim3d err = Tij_estimate * Tji_;
        Eigen::Matrix<double, 7, 1> err_;
        err_ = err.log();

        Eigen::Matrix<double, 7, 7> Jac_i;
        Eigen::Matrix<double, 7, 7> Jac_j;
        Eigen::Matrix<double, 7, 7> Jl = Eigen::Matrix<double, 7, 7>::Zero();

        Jl.block<3, 3>(0, 0) = Sophus::RxSO3d::hat(err_.tail(4));
        Jl.block<3, 3>(0, 3) = Sophus::SO3d::hat(err_.head(3));
        Jl.block<3, 1>(0, 6) = -err_.head(3);
        Jl.block<3, 3>(3, 3) = Sophus::SO3d::hat(err_.block<3, 1>(3, 0));
        Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
        Jl.noalias() = sqrt_information_ * (I - 0.5 * Jl + 1.0/12. * (Jl * Jl));

        err_ = sqrt_information_ * err_;

        Jac_i = Jl;
        Jac_j = -Jl * Tij_estimate.Adj();
        int k = 0;
        for(int i = 0; i < 7; i++) {
            residuals[i] = err_[i];
            if(jacobians) {
                for (int j = 0; j < 7; ++j) {
                    if (jacobians[0])
                        jacobians[0][k] = Jac_j(i, j);
                    if (jacobians[1])
                        jacobians[1][k] = Jac_i(i, j);
                    k++;
                }
            }
        }
        return true;
    }

private:
    int id;
    Sophus::Sim3d Tji_;
    Eigen::Matrix<double, 7, 7> sqrt_information_;
};




Sphere::Sphere()
{
}


class CERES_EXPORT Sim3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~Sim3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 7; }
};

bool Sim3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 7, 7) = ceres::Matrix::Identity(7, 7);
    return true;
}

bool Sim3Parameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta_lie(delta);
    Sophus::Sim3d T = Sophus::Sim3d::exp(lie);
    Sophus::Sim3d delta_T = Sophus::Sim3d::exp(delta_lie);
    Eigen::Matrix<double, 7, 1> x_plus_delta_lie = (delta_T * T).log();
    for(int i = 0; i < 7; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);
    return true;
}


bool Sphere::optimize(int iter_) {
    if(vertexes.empty() == true || edges.empty() == true)
        return false;

    ceres::Problem problem;
    for(size_t m = 0; m < edges.size(); ++m) {
        ceres::CostFunction* costFun = new PoseGraphError(m, edges[m].pose, edges[m].infomation);
        // problem.AddResidualBlock(costFun, new ceres::HuberLoss(1.5), vertexes[edges[m].j].data(),
        problem.AddResidualBlock(costFun, nullptr, vertexes[edges[m].j].data(),
                                 vertexes[edges[m].i].data());
    }

    for(auto& it : vertexes) {
        problem.SetParameterization(it.second.data(), new Sim3Parameterization());
    }

    problem.SetParameterBlockConstant(vertexes[61].data());


    //printf("optimization start!\n");

    ceres::Solver::Options options;
    options.max_num_iterations = iter_;
    options.minimizer_progress_to_stdout = true;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
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



