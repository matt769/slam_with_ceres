//
// Created by matt on 04/10/2020.
//

#ifndef CERES_SIMSLAM_POSE_H
#define CERES_SIMSLAM_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class Pose {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_;
    Eigen::Quaterniond q_;
    Pose(Eigen::Vector3d p, Eigen::Quaterniond q);
    Pose();
    Pose(Eigen::Vector3d p);
    Pose(Eigen::Quaterniond q);
    Pose inverse() const ;
};

Pose operator*(const Pose& a, const Pose& b);

// Use these interchangeably
using RelativeMotion = Pose;

#endif //CERES_SIMSLAM_POSE_H
