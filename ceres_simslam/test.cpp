//
// Created by matt on 13/10/2020.
//

#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include "cost_functions.h"
#include "pose.h"
#include "graph.h"

bool comparePoses(const Pose& a, const Pose& b) {
    constexpr double tol = 0.0000001;
    return std::abs(a.p_.x() - b.p_.x()) < tol
           && std::abs(a.p_.y() - b.p_.y()) < tol
           && std::abs(a.p_.z() - b.p_.z()) < tol
           && std::abs(a.q_.x() - b.q_.x()) < tol
           && std::abs(a.q_.y() - b.q_.y()) < tol
           && std::abs(a.q_.z() - b.q_.z()) < tol
           && std::abs(a.q_.w() - b.q_.w()) < tol;
};
/** @brief Test optimisation with 2 nodes and 1 (parameterised) edge
 *
 * @param motion
 * @return true
 */
bool testOptimisation(const RelativeMotion& motion) {
    // little optimisation test with 2 nodes (1 fixed) and a single edge
    Node start{0, Pose()};
//    RelativeMotion motion(Eigen::Vector3d::UnitX(), Eigen::Quaterniond::Identity());
    Pose starting_position(motion.p_ * 1.5, Eigen::Quaterniond::Identity());
    Node end {1, starting_position};
    Edge edge{0, 1, motion, Eigen::Matrix<double, 6, 6>::Identity()};

    // What is the distance between the two nodes before optimisation
    std::cout << "Observed (true) edge: " << motion.p_.transpose() << '\n';
    std::cout << "Node distance before optimisation: " << (end.pose_.p_ - start.pose_.p_).transpose() << '\n';

    // create Ceres problem and optimise
    ceres::Problem problem;
    ceres::LossFunction* loss_function = nullptr;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    ceres::CostFunction* cost_function = RelativeMotionCost::Create(edge);
    problem.AddResidualBlock(cost_function, loss_function,
                             start.pose_.p_.data(), start.pose_.q_.coeffs().data(),
                             end.pose_.p_.data(), end.pose_.q_.coeffs().data());
    problem.SetParameterization(start.pose_.q_.coeffs().data(),
                                quaternion_local_parameterization);
    problem.SetParameterization(end.pose_.q_.coeffs().data(),
                                quaternion_local_parameterization);

    problem.SetParameterBlockConstant(start.pose_.p_.data());
    problem.SetParameterBlockConstant(start.pose_.q_.coeffs().data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

//    std::cout << summary.FullReport() << '\n';

    // What is the distance between the two nodes now?
    std::cout << "Node distance after optimisation: " << (end.pose_.p_ - start.pose_.p_).transpose() << '\n';

    return true;
}


int main(int /*argc*/, char* argv[]) {
    google::InitGoogleLogging(argv[0]);

    {
        Pose pose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
        Pose new_pose = pose.inverse().inverse();
        CHECK(comparePoses(pose, new_pose));
    }
    {
        Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity());
        Pose new_pose = pose.inverse().inverse();
        CHECK(comparePoses(pose, new_pose));
    }
    {
        Eigen::Quaterniond q(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()));
        Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0), q);
        Pose new_pose = pose.inverse().inverse();
        CHECK(comparePoses(pose, new_pose));
    }

    testOptimisation(RelativeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
    testOptimisation(RelativeMotion(Eigen::Vector3d(0.0, 1.0, 0.0)));
    testOptimisation(RelativeMotion(Eigen::Vector3d(0.0, 0.0, 1.0)));
    testOptimisation(RelativeMotion(Eigen::Vector3d(1.0, 1.0, 1.0)));
    testOptimisation(RelativeMotion(Eigen::Vector3d(-1.0, -1.0, -1.0)));



    return 0;
}