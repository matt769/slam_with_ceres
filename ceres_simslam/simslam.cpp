//
// Created by matt on 04/10/2020.
//

#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "pose.h"
#include "simulator.h"

int main(int /*argc*/, char* argv[]) {
    google::InitGoogleLogging(argv[0]);

    // just move around in xy plane, no rotation
    const Eigen::Vector3d forward(1.0, 0.0, 0.0);
    const Eigen::Vector3d backward = -forward;
    const Eigen::Vector3d left(0.0, 1.0, 0.0);
    const Eigen::Vector3d right = -left;

    Simulator::Noise noise;
    noise.mean = Eigen::Matrix<double, 6, 1>::Zero();
    noise.std_dev = Eigen::Matrix<double, 6, 1>::Zero();
    noise.std_dev(0) = 0.2; // x
    noise.std_dev(1) = 0.2; // y
    Simulator::Drift drift;
    drift.p_ = Eigen::Vector3d::Zero();
    drift.q_ = Eigen::Quaterniond::Identity();
    Simulator simulator(noise, drift);
    simulator.addFirstNode(Pose());

    constexpr size_t steps_fw_bw = 10;
    constexpr size_t steps_left_right = 6;
    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        RelativeMotion motion(forward, Eigen::Quaterniond::Identity());
        simulator.addMotionEdge(motion);
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        RelativeMotion motion(left, Eigen::Quaterniond::Identity());
        simulator.addMotionEdge(motion);
    }
    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        RelativeMotion motion(backward, Eigen::Quaterniond::Identity());
        simulator.addMotionEdge(motion);
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        RelativeMotion motion(right, Eigen::Quaterniond::Identity());
        simulator.addMotionEdge(motion);
    }

    // create a loop closure at the end
    simulator.addLoopClosure();

    // output graph current state
    std::ofstream output_file;
    output_file.open("true_poses.txt");
    output_file << simulator.getGroundTruth().toString();

    std::ofstream output_file_noisy;
    output_file_noisy.open("noisy_poses.txt");
    output_file_noisy << simulator.getGraph().toString();

    simulator.optimiseGraph();

    // output graph optimised state
    std::ofstream output_file_opt;
    output_file_opt.open("optimised_poses.txt");
    output_file_opt << simulator.getGraph().toString();

    // compare true and optimised

    // total trajectory distance (simple metric)



    return 0;
}

// TODO find out why my static function for generating node ids caused a compilation error
//  (when filling vector?)