//
// Created by matt on 13/10/2020.
//

#ifndef CERES_SIMSLAM_GRAPH_H
#define CERES_SIMSLAM_GRAPH_H

#include <cstddef>

#include "pose.h"

struct Edge {
    size_t start;
    size_t end;
    RelativeMotion relative_motion;
};

class Node {
public:
    size_t id_;
    Pose pose_;
//    explicit Node(Pose pose) : id_(Node::nextId()), pose_(std::move(pose)) {};
//    Node() : Node(Pose()) {};
//    static size_t nextId() { return next_id++; }
//private:
//    static size_t next_id;
};

#endif //CERES_SIMSLAM_GRAPH_H
