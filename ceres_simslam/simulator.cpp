//
// Created by matt on 16/10/2020.
//

#include "simulator.h"

#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include "pose.h"
#include "graph.h"

namespace simulator {
using namespace pose;
using namespace graph;

Simulator::Simulator()
    : Simulator(Noise{Eigen::Matrix<double, 6, 1>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), 0.0, 0.0}, Drift()) {}

Simulator::Simulator(Noise noise, Drift drift)
        : drift_(drift), noise_generator_(0)
{
    noise_distribution_.reserve(6);
    setNoise(noise);

    Eigen::Matrix<double, 3, 3> orientation_cov = Eigen::Matrix<double, 3, 3>::Zero();
    orientation_cov.diagonal() = Eigen::Vector3d(noise_.orientation.std_dev, noise_.orientation.std_dev, noise_.orientation.std_dev);
    orientation_cov = orientation_cov + Eigen::Matrix<double, 3, 3>::Identity();
    orientation_sqrt_info_ = orientation_cov.inverse().sqrt();

    Eigen::Matrix<double, 3, 3> gravity_cov = Eigen::Matrix<double, 3, 3>::Zero();
    // Just re-using the orientation noise
    gravity_cov.diagonal() = Eigen::Vector3d(noise_.orientation.std_dev, noise_.orientation.std_dev, noise_.orientation.std_dev);
    gravity_cov = gravity_cov + Eigen::Matrix<double, 3, 3>::Identity();
    gravity_sqrt_info_ = gravity_cov.inverse().sqrt();

    Eigen::Matrix<double, 6, 6> relative_motion_cov = Eigen::Matrix<double, 6, 6>::Zero();
    relative_motion_cov.diagonal() << noise_.relative_motion.std_dev, noise_.relative_motion.std_dev,
                                        noise_.relative_motion.std_dev, noise_.relative_motion.std_dev,
                                        noise_.relative_motion.std_dev, noise_.relative_motion.std_dev;
    relative_motion_cov = relative_motion_cov + Eigen::Matrix<double, 6, 6>::Identity();
    relative_motion_sqrt_info_ = relative_motion_cov.inverse().sqrt();

    setMeasurableFixedFrame(Eigen::Quaterniond::Identity());
    gravity_ = Eigen::Vector3d::UnitZ();
}

void Simulator::setNoise(Noise noise) {
    noise_ = noise;
    for (size_t idx = 0; idx < 6; ++idx) {
        noise_distribution_[idx] = std::normal_distribution<double>(noise_.relative_motion.mean(idx), noise_.relative_motion.std_dev(idx));
    }
}

void Simulator::addFirstNode(const Pose& pose) {
    graph_.addFirstNode(pose);
    ground_truth_.addFirstNode(pose);
}

void Simulator::addMotionEdge(const RelativeMotion& motion) {
    ground_truth_.addMotionEdge(motion, Eigen::Matrix<double, 6, 6>::Identity());
    graph_.addMotionEdge(addNoise(addDrift(motion)), relative_motion_sqrt_info_);
}

void Simulator::addOrientationEdge() {
    // q_node_fixedframe =  inverse(q_world_node) * q_world_fixedframe
    const Eigen::Quaterniond measurement = ground_truth_.getLastNode().pose_.q_.conjugate() * measurable_fixed_frame_;
    ground_truth_.addOrientationEdge(measurement, Eigen::Matrix<double, 3, 3>::Identity());
    graph_.addOrientationEdge(addNoise(measurement), orientation_sqrt_info_);
}

void Simulator::addGravityEdge() {
    const Eigen::Vector3d measurement = ground_truth_.getLastNode().pose_.q_.conjugate() * gravity_;
    ground_truth_.addGravityEdge(measurement, Eigen::Matrix<double, 3, 3>::Identity());
    graph_.addGravityEdge(addNoise(measurement), gravity_sqrt_info_);
}

void Simulator::addLoopClosure() {
    // atm fixed to be a beginning/end loop closure
    addLoopClosure(ground_truth_.getLastNodeId(), 0);
}

void Simulator::addLoopClosure(const size_t start, const size_t end) {
    // Uses the same noise/information matrix as for the motion edges
    // generate the ground truth
    const Node loop_start_node_gt = ground_truth_.getNodes()[start];
    const Node loop_end_node_gt = ground_truth_.getNodes()[end];
    const RelativeMotion loop_pseudo_motion = loop_start_node_gt.pose_.inverse() * loop_end_node_gt.pose_;
    ground_truth_.addLoopClosureEdge(loop_start_node_gt.id_, loop_end_node_gt.id_, loop_pseudo_motion, Eigen::Matrix<double, 6, 6>::Identity());
    // then add noise
    RelativeMotion T_end_start_noisy = addNoise(loop_pseudo_motion);
    graph_.addLoopClosureEdge(loop_start_node_gt.id_, loop_end_node_gt.id_, T_end_start_noisy, relative_motion_sqrt_info_);
}

void Simulator::setMeasurableFixedFrame(const Eigen::Quaterniond& q) {
    measurable_fixed_frame_ = q;
}

bool Simulator::optimiseGraph() {
    return graph_.optimise();
}

void Simulator::compare() const {

}

RelativeMotion Simulator::addNoise(const RelativeMotion& motion) {
    RelativeMotion noisy_motion(motion);
    noisy_motion.p_.x() += noise_distribution_[0](noise_generator_);
    noisy_motion.p_.y() += noise_distribution_[1](noise_generator_);
    noisy_motion.p_.z() += noise_distribution_[2](noise_generator_);
    noisy_motion.q_ *= generateRandomRotation(noise_.relative_motion.std_dev[3]);
    return noisy_motion;
}

Eigen::Quaterniond Simulator::addNoise(const Eigen::Quaterniond& rotation) {
    Eigen::Quaterniond noisy_rotation = rotation;
    noisy_rotation *= generateRandomRotation(noise_.orientation.std_dev);
    return noisy_rotation;
}

// TODO using the noise generator for the relative motion for temporary convenience
Eigen::Vector3d Simulator::addNoise(const Eigen::Vector3d &v) {
    Eigen::Vector3d noisy_vector = v;
    noisy_vector.x() += noise_distribution_[0](noise_generator_);
    noisy_vector.y() += noise_distribution_[1](noise_generator_);
    noisy_vector.z() += noise_distribution_[2](noise_generator_);
    return noisy_vector;
}

RelativeMotion Simulator::addDrift(const RelativeMotion& motion) {
    RelativeMotion drifted_motion(motion);
    drifted_motion.p_ += drift_.p_;
    drifted_motion.q_ *= drift_.q_;
    return drifted_motion;
}

const Graph& Simulator::getGroundTruth() const {
    return ground_truth_;
}

const Graph& Simulator::getGraph() const {
    return graph_;
}

Eigen::Matrix<double, 6, 6> Simulator::toSqrtInfo(const Noise& noise) const {
    // not sure if this is really the right approach but
    // because we can have zero noise, need to be able to 'invert' a matrix of zeros
    // So just add identity to the noise
    Eigen::Matrix<double, 6, 1> modified_noise = noise.relative_motion.std_dev + Eigen::Matrix<double, 6, 1>::Ones();
    return modified_noise.asDiagonal().toDenseMatrix().sqrt();
}

Eigen::Quaterniond Simulator::generateRandomRotation(const double angle_stddev) {
    Eigen::Vector3d axis;
    axis.setRandom().normalize();
    auto distribution = std::normal_distribution<double>(0.0, angle_stddev);
    double angle = distribution(noise_generator_);
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

} // namespace simulator