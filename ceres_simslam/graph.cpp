//
// Created by matt on 16/10/2020.
//

#include "graph.h"

#include <cstddef>
#include <sstream>

#include <glog/logging.h>
#include <ceres/ceres.h>

#include "pose.h"
#include "cost_functions.h"


Graph::Graph() : next_id_(0) {}

void Graph::addFirstNode(const Pose& pose) {
    CHECK(next_id_ == 0) << "Graph already contains " << next_id_ - 1 << " nodes!";;
    addNode(pose);
}

void Graph::addMotionEdge(const RelativeMotion& motion) {
    CHECK(next_id_ > 0) << "Graph needs to contain at least 1 node first!";;
    Pose new_pose = nodes_.back().pose_ * motion;
    addNode(new_pose);
    addEdge(getLastNodeId()-1, getLastNodeId(), motion);
}

void Graph::addLoopClosureEdge(const size_t start, const size_t end, const RelativeMotion& motion) {
    CHECK(start < nodes_.size()) << "Starting node id " << start << " not present in node list!";
    CHECK(end < nodes_.size()) << "Starting node id " << end << " not present in node list!";
    addEdge(start, end, motion);
}

size_t Graph::getLastNodeId() const {
    CHECK(next_id_ > 0) << "Graph is empty!";
    return next_id_ - 1;
}

const Node& Graph::getNode(const size_t node_id) const {
    CHECK(next_id_ > 0) << "Graph is empty!";
    CHECK(node_id < nodes_.size()) << "Node id " << node_id << " not present in node list!";
    return nodes_[node_id];
}

const std::vector<Node>& Graph::getNodes() const {
    return nodes_;
}

const Node& Graph::getLastNode() const {
    return nodes_[getLastNodeId()];
}

size_t Graph::addNode(const Pose& pose) {
    nodes_.emplace_back(Node{next_id_++, pose});
    return getLastNodeId();
}

void Graph::addEdge(const size_t start, const size_t end, const RelativeMotion& motion) {
    CHECK(start < nodes_.size()) << "Starting node id " << start << " not present in node list!";
    CHECK(end < nodes_.size()) << "Starting node id " << end << " not present in node list!";
    edges_.emplace_back(Edge{start, end, motion});
}

bool Graph::optimise() {
    CHECK(next_id_ > 0) << "Graph is empty!";
    CHECK(next_id_ != 1) << "Graph only contains a single node!";

    ceres::Problem problem;
    ceres::LossFunction* loss_function = nullptr;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    for (const auto& edge: edges_) {
        ceres::CostFunction* cost_function = RelativeMotionCost::Create(edge.relative_motion);
        problem.AddResidualBlock(cost_function, loss_function,
                                 nodes_[edge.start].pose_.p_.data(), nodes_[edge.start].pose_.q_.coeffs().data(),
                                 nodes_[edge.end].pose_.p_.data(), nodes_[edge.end].pose_.q_.coeffs().data());
        problem.SetParameterization(nodes_[edge.start].pose_.q_.coeffs().data(),
                                    quaternion_local_parameterization);
        problem.SetParameterization(nodes_[edge.end].pose_.q_.coeffs().data(),
                                    quaternion_local_parameterization);
    }

    problem.SetParameterBlockConstant(nodes_[0].pose_.p_.data());
    problem.SetParameterBlockConstant(nodes_[0].pose_.q_.coeffs().data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return summary.IsSolutionUsable();
}

std::string Graph::toString() const {
    std::stringstream ss;
    ss << "Nodes (" << nodes_.size() << "):\n";
    ss << nodesToString();
    ss << "Edges (" << edges_.size() << "):\n";
    ss << edgesToString();
    return ss.str();
}

std::string Graph::nodesToString() const {
    std::stringstream ss;
    for (const auto& node: nodes_) {
        ss << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }
    return ss.str();
}

std::string Graph::edgesToString() const {
    std::stringstream ss;
    for (const auto& edge: edges_) {
        ss << edge.start << ' ' << edge.end << ' ' << edge.relative_motion.p_.transpose() << ' ' << edge.relative_motion.q_.coeffs().transpose() << '\n';
    }
    return ss.str();
}