//
// Created by matt on 04/10/2020.
//

#ifndef CERES_SIMSLAM_POSE_H
#define CERES_SIMSLAM_POSE_H

// maybe I should use transformation matrices instead of this pose, and then just convert when I want to optimise?

// check that quaternion/vector * operator act likes I expect

class Pose {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_;
    Eigen::Quaterniond q_;
    Pose(Eigen::Vector3d p, Eigen::Quaterniond q) : p_(p), q_(q) {};
    Pose() : Pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()) {};
    Pose(Eigen::Vector3d p) : Pose(p, Eigen::Quaterniond::Identity()) {};
    Pose(Eigen::Quaterniond q) : Pose(Eigen::Vector3d::Zero(), q) {};
    Pose inverse() { return Pose(q_.conjugate() * -p_, q_.conjugate()); }
};

Pose operator*(const Pose& a, const Pose& b) {
    return Pose(a.p_ + a.q_ * b.p_, a.q_ * b.q_);
}

// Use these interchangeably
using RelativeMotion = Pose;

#endif //CERES_SIMSLAM_POSE_H
