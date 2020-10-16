//
// Created by matt on 16/10/2020.
//

#include "pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

Pose::Pose(Eigen::Vector3d p, Eigen::Quaterniond q) : p_(p), q_(q) {};

Pose::Pose() : Pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()) {};

Pose::Pose(Eigen::Vector3d p) : Pose(p, Eigen::Quaterniond::Identity()) {};

Pose::Pose(Eigen::Quaterniond q) : Pose(Eigen::Vector3d::Zero(), q) {};

Pose Pose::inverse() { return Pose(q_.conjugate() * -p_, q_.conjugate()); }

Pose operator*(const Pose& a, const Pose& b) {
    return Pose(a.p_ + a.q_ * b.p_, a.q_ * b.q_);
}
