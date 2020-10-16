//
// Created by matt on 16/10/2020.
//

#include "simulator.h"

#include <random>

#include "pose.h"
#include "graph.h"


Simulator::Simulator()
    : Simulator(Noise{0.0, 0.0}) {}

Simulator::Simulator(Noise noise)
        : noise_(noise),
          noise_generator_(0),
          noise_distribution_(noise_.mean, noise_.std_dev)
          {}

void Simulator::setNoise(double mean, double std_dev) {
    noise_.mean = mean;
    noise_.std_dev = std_dev;
    noise_distribution_ = std::normal_distribution<double>(noise_.mean, noise_.std_dev);
}

void Simulator::addFirstNode(const Pose& pose) {
    graph_.addFirstNode(pose);
    ground_truth_.addFirstNode(pose);
}

void Simulator::addMotionEdge(const RelativeMotion& motion) {
    ground_truth_.addMotionEdge(motion);
    graph_.addMotionEdge(addNoise(motion));
}

void Simulator::addLoopClosure() {
    // atm fixed to be a beginning/end loop closure
    addLoopClosure(ground_truth_.getLastNodeId(), 0);
}

void Simulator::addLoopClosure(const size_t start, const size_t end) {
    // generate the ground truth
    const Node loop_start_node_gt = ground_truth_.getNodes()[start];
    const Node loop_end_node_gt = ground_truth_.getNodes()[end];
    const RelativeMotion loop_pseudo_motion = loop_start_node_gt.pose_.inverse() * loop_end_node_gt.pose_;
    ground_truth_.addLoopClosureEdge(loop_start_node_gt.id_, loop_end_node_gt.id_, loop_pseudo_motion);
    // then add noise
    RelativeMotion T_end_start_noisy = addNoise(loop_pseudo_motion);
    graph_.addLoopClosureEdge(loop_start_node_gt.id_, loop_end_node_gt.id_, T_end_start_noisy);
}

bool Simulator::optimiseGraph() {
    graph_.optimise();
}

void Simulator::compare() const {

}

RelativeMotion Simulator::addNoise(const RelativeMotion& motion) {
    RelativeMotion noisy_motion(motion);
    noisy_motion.p_.x() += noise_distribution_(noise_generator_);
    noisy_motion.p_.y() += noise_distribution_(noise_generator_);
    noisy_motion.p_.z() = motion.p_.z();
    return noisy_motion;
}

const Graph& Simulator::getGroundTruth() const {
    return ground_truth_;
}

const Graph& Simulator::getGraph() const {
    return graph_;
}