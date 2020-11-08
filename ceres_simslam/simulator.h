//
// Created by matt on 16/10/2020.
//

#ifndef CERES_SIMSLAM_SIMULATOR_H
#define CERES_SIMSLAM_SIMULATOR_H

#include <random>
#include <vector>

#include <Eigen/Core>

#include "graph.h"
#include "pose.h"

class Simulator {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    struct Noise {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<double, 6, 1> mean;
        Eigen::Matrix<double, 6, 1> std_dev;
    };
    typedef Pose Drift;
    Simulator();
    Simulator(Noise noise, Drift drift);
    void addFirstNode(const Pose& pose);
    void addMotionEdge(const RelativeMotion& motion);
    void addOrientationEdge();
    void addLoopClosure();
    void addLoopClosure(size_t start, size_t end);
    void setMeasurableFixedFrame(const Eigen::Quaterniond& q);
    bool optimiseGraph();
    void compare() const;
    const Graph& getGroundTruth() const;
    const Graph& getGraph() const;
private:
    Graph ground_truth_;
    Graph graph_;
    Noise noise_;
    Drift drift_;
    std::default_random_engine noise_generator_;
    std::vector<std::normal_distribution<double>> noise_distribution_;
    Eigen::Quaterniond measurable_fixed_frame_;
    void setNoise(Noise noise);
    RelativeMotion addNoise(const RelativeMotion& motion);
    Eigen::Quaterniond addNoise(const Eigen::Quaterniond& rotation);
    RelativeMotion addDrift(const RelativeMotion& motion);
    Eigen::Matrix<double, 6, 6> toSqrtInfo(const Noise& noise) const;
    Eigen::Quaterniond generateRandomRotation(double angle_stddev);
};


#endif //CERES_SIMSLAM_SIMULATOR_H
