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

namespace graph {
using namespace pose;

template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

Graph::Graph() : next_id_(0) {}

void Graph::addFirstNode(const Pose &pose) {
    CHECK(next_id_ == 0) << "Graph already contains " << next_id_ - 1 << " nodes!";;
    addNode(pose);
}

void Graph::addMotionEdge(const RelativeMotion &motion, const Eigen::Matrix<double, 6, 6> &sqrt_info) {
    CHECK(next_id_ > 0) << "Graph needs to contain at least 1 node first!";;
    Pose new_pose = nodes_.back().pose_ * motion;
    addNode(new_pose);
    addEdge(getLastNodeId() - 1, getLastNodeId(), motion, sqrt_info);
}

void Graph::addOrientationEdge(const Eigen::Quaterniond &measurement, const Eigen::Matrix<double, 3, 3> &sqrt_info) {
    CHECK(next_id_ > 0) << "Graph needs to contain at least 1 node first!";;
    orientation_edges_.emplace_back(OrientationEdge{getLastNodeId(), measurement, sqrt_info});
}

void Graph::addGravityEdge(const Eigen::Vector3d &measurement, const Eigen::Matrix<double, 3, 3> &sqrt_info) {
    CHECK(next_id_ > 0) << "Graph needs to contain at least 1 node first!";;
    orientation_edges_.emplace_back(GravityEdge{getLastNodeId(), measurement, sqrt_info});
}

void Graph::addAbsolutePositionEdge(const Eigen::Vector3d &measurement, const Eigen::Matrix<double, 3, 3> &sqrt_info) {
    CHECK(next_id_ > 0) << "Graph needs to contain at least 1 node first!";;
    absolute_position_edges_.emplace_back(AbsolutePositionEdge{getLastNodeId(), measurement, sqrt_info});
}

void Graph::addLoopClosureEdge(const size_t start, const size_t end, const RelativeMotion &motion,
                               const Eigen::Matrix<double, 6, 6> &sqrt_info) {
    CHECK(start < nodes_.size()) << "Starting node id " << start << " not present in node list!";
    CHECK(end < nodes_.size()) << "Starting node id " << end << " not present in node list!";
    addEdge(start, end, motion, sqrt_info);
}

size_t Graph::getLastNodeId() const {
    CHECK(next_id_ > 0) << "Graph is empty!";
    return next_id_ - 1;
}

const Node &Graph::getNode(const size_t node_id) const {
    CHECK(next_id_ > 0) << "Graph is empty!";
    CHECK(node_id < nodes_.size()) << "Node id " << node_id << " not present in node list!";
    return nodes_[node_id];
}

const std::vector<Node> &Graph::getNodes() const {
    return nodes_;
}

const Node &Graph::getLastNode() const {
    return nodes_[getLastNodeId()];
}

size_t Graph::addNode(const Pose &pose) {
    nodes_.emplace_back(Node{next_id_++, pose});
    return getLastNodeId();
}

void Graph::addEdge(const size_t start, const size_t end, const RelativeMotion &motion,
                    const Eigen::Matrix<double, 6, 6> &sqrt_info) {
    CHECK(start < nodes_.size()) << "Starting node id " << start << " not present in node list!";
    CHECK(end < nodes_.size()) << "Starting node id " << end << " not present in node list!";
    edges_.emplace_back(RelativeMotionEdge{start, end, motion, sqrt_info});
}

bool Graph::optimise() {
    CHECK(next_id_ > 0) << "Graph is empty!";
    CHECK(next_id_ != 1) << "Graph only contains a single node!";

    ceres::Problem problem;
    ceres::LossFunction *loss_function = nullptr;
    ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    for (const auto &edge: edges_) {
        ceres::CostFunction *cost_function = RelativeMotionCost::Create(edge);
        problem.AddResidualBlock(cost_function, loss_function,
                                 nodes_[edge.start].pose_.p_.data(), nodes_[edge.start].pose_.q_.coeffs().data(),
                                 nodes_[edge.end].pose_.p_.data(), nodes_[edge.end].pose_.q_.coeffs().data());
        problem.SetParameterization(nodes_[edge.start].pose_.q_.coeffs().data(),
                                    quaternion_local_parameterization);
        problem.SetParameterization(nodes_[edge.end].pose_.q_.coeffs().data(),
                                    quaternion_local_parameterization);
    }

    for (const auto &edge: orientation_edges_) {
        std::visit(overload {
                [&](const OrientationEdge& o_edge) {
                    ceres::CostFunction *cost_function = OrientationCost::Create(o_edge);
                    problem.AddResidualBlock(cost_function, loss_function,
                                             nodes_[o_edge.node_id].pose_.q_.coeffs().data());
                    problem.SetParameterization(nodes_[o_edge.node_id].pose_.q_.coeffs().data(),
                                                quaternion_local_parameterization);
                },
                [&](const GravityEdge& g_edge) {
                    ceres::CostFunction *cost_function = GravityCost::Create(g_edge);
                    problem.AddResidualBlock(cost_function, loss_function,
                                             nodes_[g_edge.node_id].pose_.q_.coeffs().data());
                    problem.SetParameterization(nodes_[g_edge.node_id].pose_.q_.coeffs().data(),
                                                quaternion_local_parameterization);
                }
            },
           edge);
    }

    for (const auto &edge: absolute_position_edges_) {
        ceres::CostFunction *cost_function = AbsolutePositionCost::Create(edge);
        problem.AddResidualBlock(cost_function, loss_function,
                                 nodes_[edge.node_id].pose_.p_.data());
    }

    problem.SetParameterBlockConstant(nodes_[0].pose_.p_.data());
    problem.SetParameterBlockConstant(nodes_[0].pose_.q_.coeffs().data());
    // TODO release fixed first node


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
//    ss << "OrientationEdges (" << orientation_edges_.size() << "):\n";
//    ss << orientationEdgesToString();
    return ss.str();
}

std::string Graph::nodesToString() const {
    std::stringstream ss;
    for (const auto &node: nodes_) {
        ss << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }
    return ss.str();
}

std::string Graph::edgesToString() const {
    std::stringstream ss;
    for (const auto &edge: edges_) {
        ss << edge.start << ' ' << edge.end << ' ' << edge.relative_motion.p_.transpose() << ' '
           << edge.relative_motion.q_.coeffs().transpose() << '\n';
    }
    return ss.str();
}

// TODO add explicit indication of which type it is?
std::string Graph::orientationEdgesToString() const {
    std::stringstream ss;
    for (const auto& edge: orientation_edges_) {
        std::visit(overload {
           [&](const OrientationEdge& o_edge) { ss << o_edge.node_id << ' ' << o_edge.orientation.coeffs().transpose() << '\n';  },
           [&](const GravityEdge& g_edge) { ss << g_edge.node_id << ' ' << g_edge.gravity_vector.transpose() << '\n'; }
           },
           edge);
    }
    return ss.str();
}

} // namespace graph