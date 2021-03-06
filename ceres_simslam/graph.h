//
// Created by matt on 13/10/2020.
//

#ifndef CERES_SIMSLAM_GRAPH_H
#define CERES_SIMSLAM_GRAPH_H

#include <cstddef>
#include <vector>
#include <variant>

#include <Eigen/Core>

#include "pose.h"

namespace graph {

struct RelativeMotionEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t start;
    size_t end;
    pose::RelativeMotion relative_motion;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};

struct Node {
    size_t id_;
    pose::Pose pose_;
};

struct OrientationEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t node_id;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double, 3, 3> sqrt_info;
};

struct GravityEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t node_id;
    Eigen::Vector3d gravity_vector;
    Eigen::Matrix<double, 3, 3> sqrt_info;
};

struct AbsolutePositionEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t node_id;
    Eigen::Vector3d position;
    Eigen::Matrix<double, 3, 3> sqrt_info;
};

class Graph {
public:
    Graph();

    void addFirstNode(const pose::Pose &pose);

    void addMotionEdge(const pose::RelativeMotion &motion, const Eigen::Matrix<double, 6, 6> &sqrt_info);

    void addOrientationEdge(const Eigen::Quaterniond &measurement, const Eigen::Matrix<double, 3, 3> &sqrt_info);

    void addGravityEdge(const Eigen::Vector3d &measurement, const Eigen::Matrix<double, 3, 3> &sqrt_info);

    void addAbsolutePositionEdge(const Eigen::Vector3d &measurement, const Eigen::Matrix<double, 3, 3> &sqrt_info);

    void addLoopClosureEdge(size_t start, size_t end, const pose::RelativeMotion &motion,
                            const Eigen::Matrix<double, 6, 6> &sqrt_info);

    size_t getLastNodeId() const;

    const Node &getNode(size_t node_id) const;

    const std::vector<Node> &getNodes() const;

    const Node &getLastNode() const;

    bool optimise();

    std::string toString() const;

    std::string nodesToString() const;

    std::string edgesToString() const;

    std::string orientationEdgesToString() const;

private:
    size_t addNode(const pose::Pose &pose);

    void
    addEdge(size_t start, size_t end, const pose::RelativeMotion &motion, const Eigen::Matrix<double, 6, 6> &sqrt_info);

    std::vector<Node> nodes_;
    std::vector<RelativeMotionEdge> edges_;
    std::vector<std::variant<OrientationEdge, GravityEdge>> orientation_edges_;
    std::vector<AbsolutePositionEdge> absolute_position_edges_;
    size_t next_id_;
};

} // namespace graph

#endif //CERES_SIMSLAM_GRAPH_H
