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


Simulator::Simulator()
    : Simulator(Noise{Eigen::Matrix<double, 6, 1>::Zero(), Eigen::Matrix<double, 6, 1>::Zero()}, Drift()) {}

Simulator::Simulator(Noise noise, Drift drift)
        : drift_(drift), noise_generator_(0)
{
    noise_distribution_.reserve(6);
    setNoise(noise);
}

void Simulator::setNoise(Noise noise) {
    noise_ = noise;
    for (size_t idx = 0; idx < 6; ++idx) {
        noise_distribution_[idx] = std::normal_distribution<double>(noise_.mean(idx), noise_.std_dev(idx));
    }
}

void Simulator::addFirstNode(const Pose& pose) {
    graph_.addFirstNode(pose);
    ground_truth_.addFirstNode(pose);
}

void Simulator::addMotionEdge(const RelativeMotion& motion) {
    ground_truth_.addMotionEdge(motion, Eigen::Matrix<double, 6, 6>::Identity());
    graph_.addMotionEdge(addNoise(addDrift(motion)), toSqrtInfo(noise_));
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
    graph_.addLoopClosureEdge(loop_start_node_gt.id_, loop_end_node_gt.id_, T_end_start_noisy, toSqrtInfo(noise_));
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
    // TODO add noise quaternion
    return noisy_motion;
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
    Eigen::Matrix<double, 6, 1> modified_noise = noise.std_dev + Eigen::Matrix<double, 6, 1>::Ones();
    return modified_noise.asDiagonal().toDenseMatrix().sqrt();
}