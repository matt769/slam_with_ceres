//
// Created by matt on 04/10/2020.
//

#include <vector>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

// start off simple, following pose_graph_3d but with my own generated data
// then try using eigen structures
// then maybe simulate the robot a bit more 'freely'?

struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Pose() : p(Eigen::Vector3d::Zero()), q(Eigen::Quaterniond::Identity()) {};
    Pose(Eigen::Vector3d p_in, Eigen::Quaterniond q_in) : p(p_in), q(q_in) {};
};

Pose operator*(const Pose& a, const Pose& b) {
    return Pose(a.p + a.q * b.p, a.q * b.q);
}

int main() {
    std::vector<Pose> true_trajectory;
    true_trajectory.emplace_back(Pose());
    std::vector<Pose> perfect_odometry;
    // just move around in xy plane, no rotation
    const Eigen::Vector3d forward(1.0, 0.0, 0.0);
    const Eigen::Vector3d backward = -forward;
    const Eigen::Vector3d left(0.0, 1.0, 0.0);
    const Eigen::Vector3d right = -left;

    constexpr size_t steps_fw_bw = 10;
    constexpr size_t steps_left_right = 5;
    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        perfect_odometry.emplace_back(Pose(forward, Eigen::Quaterniond::Identity()));
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        perfect_odometry.emplace_back(Pose(left, Eigen::Quaterniond::Identity()));
    }
    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        perfect_odometry.emplace_back(Pose(backward, Eigen::Quaterniond::Identity()));
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        perfect_odometry.emplace_back(Pose(right, Eigen::Quaterniond::Identity()));
    }

    for (const auto& movement: perfect_odometry) {
        true_trajectory.emplace_back(true_trajectory.back() * movement);
    }

    for (const auto& pose: true_trajectory) {
        std::cout << pose.p.transpose() << '\n';
    }

    //



    return 0;
}