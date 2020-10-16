//
// Created by matt on 13/10/2020.
//

#ifndef CERES_SIMSLAM_GRAPH_H
#define CERES_SIMSLAM_GRAPH_H

#include <cstddef>
#include <vector>

#include "pose.h"

struct Edge {
    size_t start;
    size_t end;
    RelativeMotion relative_motion;
};

struct Node {
    size_t id_;
    Pose pose_;
};

class Graph {
public:
    Graph();
    void addFirstNode(const Pose& pose);
    void addMotionEdge(const RelativeMotion& motion);
    void addLoopClosureEdge(size_t start, size_t end, const RelativeMotion& motion);
    size_t getLastNodeId() const;
    const Node& getNode(size_t node_id) const;
    const std::vector<Node>& getNodes() const;
    const Node& getLastNode() const;
    bool optimise();
    std::string toString() const;
    std::string nodesToString() const;
    std::string edgesToString() const;
private:
    size_t addNode(const Pose& pose);
    void addEdge(size_t start, size_t end, const RelativeMotion& motion);
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    size_t next_id_;
};

#endif //CERES_SIMSLAM_GRAPH_H
