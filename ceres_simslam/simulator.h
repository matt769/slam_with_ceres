//
// Created by matt on 16/10/2020.
//

#ifndef CERES_SIMSLAM_SIMULATOR_H
#define CERES_SIMSLAM_SIMULATOR_H

#include <random>

#include "graph.h"
#include "pose.h"

class Simulator {
public:
    struct Noise {
        double mean;
        double std_dev;
    };
    Simulator();
    Simulator(Noise noise);
    void setNoise(Noise noise);
    void addFirstNode(const Pose& pose);
    void addMotionEdge(const RelativeMotion& motion);
    void addLoopClosure();
    void addLoopClosure(size_t start, size_t end);
    bool optimiseGraph();
    void compare() const;
    const Graph& getGroundTruth() const;
    const Graph& getGraph() const;
private:
    Graph ground_truth_;
    Graph graph_;
    Noise noise_;
    std::default_random_engine noise_generator_;
    std::normal_distribution<double> noise_distribution_;
    RelativeMotion addNoise(const RelativeMotion& motion);
};


#endif //CERES_SIMSLAM_SIMULATOR_H
