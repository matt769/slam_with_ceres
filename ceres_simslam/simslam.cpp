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

using namespace pose;
using namespace simulator;

struct Args {
    simulator::Noise noise;
    simulator::Drift drift;
    enum class LoopClosureLevel {NONE, SINGLE, MANY } loopclosure_level;
    bool include_orientation_edges;
    bool include_gravity_edges;
    enum class AbsolutePositionLevel {NONE, SINGLE, TWO } absolute_position_level;
};

Args parseArgs(int argc, char* argv[]);

void printArgs(const Args& args);

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);

    Args args = parseArgs(argc, argv);

    Simulator simulator(args.noise, args.drift);
    // let's say the fixed frame we observe with our orientation measurements is identity for now
    const Eigen::Quaterniond fixed_frame = Eigen::Quaterniond::Identity();
    simulator.setMeasurableFixedFrame(fixed_frame);

    // Set a non-zero starting point if we're using absolute position measurements
    //  so that we can actually see their effect
    if (args.absolute_position_level != Args::AbsolutePositionLevel::NONE) {
        Pose absolute_frame;
        absolute_frame.p_ = Eigen::Vector3d(5.0, 5.0, 0.2);
        absolute_frame.q_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
        simulator.setAbsoluteFrame(absolute_frame);
    }

    Pose starting_pose;
    simulator.addFirstNode(starting_pose);
    if (args.absolute_position_level != Args::AbsolutePositionLevel::NONE) {
        simulator.addAbsolutePositionEdge();
    }

    constexpr size_t steps_fw_bw = 10;
    constexpr size_t steps_left_right = 6;
    const Eigen::Vector3d forward(1.0, 0.0, 0.0);
    const Eigen::Quaterniond left_turn = Eigen::Quaterniond(Eigen::AngleAxis(M_PI/2.0, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond face_forward = Eigen::Quaterniond::Identity();
    const RelativeMotion forward_motion = RelativeMotion(forward, face_forward);
    const RelativeMotion forward_and_turn_left = RelativeMotion(forward, left_turn);

    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        if (idx == steps_fw_bw - 1) {
            simulator.addMotionEdge(forward_and_turn_left);
        } else {
            simulator.addMotionEdge(forward_motion);
        }
        if (args.include_orientation_edges) simulator.addOrientationEdge();
        if (args.include_gravity_edges) simulator.addGravityEdge();
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        Eigen::Quaterniond q;
        if (idx == steps_left_right - 1) {
            simulator.addMotionEdge(forward_and_turn_left);
        } else {
            simulator.addMotionEdge(forward_motion);
        }
        if (args.include_orientation_edges) simulator.addOrientationEdge();
        if (args.include_gravity_edges) simulator.addGravityEdge();
    }

    if (args.absolute_position_level == Args::AbsolutePositionLevel::TWO) {
        simulator.addAbsolutePositionEdge();
    }

    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        Eigen::Quaterniond q;
        if (idx == steps_fw_bw - 1) {
            simulator.addMotionEdge(forward_and_turn_left);
        } else {
            simulator.addMotionEdge(forward_motion);
        }
        if (args.include_orientation_edges) simulator.addOrientationEdge();
        if (args.include_gravity_edges) simulator.addGravityEdge();
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        Eigen::Quaterniond q;
        if (idx == steps_left_right - 1) {
            simulator.addMotionEdge(forward_and_turn_left);
        } else {
            simulator.addMotionEdge(forward_motion);
        }
        if (args.include_orientation_edges) simulator.addOrientationEdge();
        if (args.include_gravity_edges) simulator.addGravityEdge();
    }

    // create a loop closure at the end
    if (args.loopclosure_level == Args::LoopClosureLevel::SINGLE) {
        simulator.addLoopClosure();
    }
    if (args.loopclosure_level == Args::LoopClosureLevel::MANY) {
        // TODO don't hard code start/end
        simulator.addLoopClosure();
        simulator.addLoopClosure(0, 16);
        simulator.addLoopClosure(10, 26);
    }

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

Args parseArgs(int argc, char* argv[]) {
    const std::string bad_args_message = "Expecting 5 arguments\n"
                                   "Include noise, none (0), motion xy only (1), motion all (2), more! (3)\n"
                                   "Include drift, none (0), xy and yaw only (1), all (2), more! (3)\n"
                                   "Include loop closure, none (0), one (1), many (2)\n"
                                   "Include orientation edges, no (0), yes (1), gravity only (2)\n"
                                   "Include absolute position measurements, no (0), one at the start (1), two - at start and middle (2)\n"
                                   "Example call:\n"
                                   "simslam 1 1 0 1 0\n";

    auto printBadArgsAndExit = [&]() {
        std::cout << bad_args_message;
        std::cout << "Received arguments:\n";
        for (int idx = 0; idx < argc; ++idx) {
            std::cout << idx << ": " << argv[idx] << '\n';
        }
        exit(1);
    };

    if (argc != 6) {
        printBadArgsAndExit();
    }

    Args args;

    try {
        switch (std::stoi(argv[1])) {
            case 0:
                args.noise.relative_motion.std_dev = Eigen::Matrix<double, 6, 1>::Zero();
                args.noise.orientation.std_dev = 0.0;
                break;
            case 1:
                args.noise.relative_motion.std_dev << 0.05, 0.05, 0.0, 0.0, 0.0, 0.0;
                args.noise.orientation.std_dev = 0.01;
                break;
            case 2:
                args.noise.relative_motion.std_dev << 0.05, 0.05, 0.01, 0.01, 0.01, 0.01;
                args.noise.orientation.std_dev = 0.01;
                break;
            case 3:
                args.noise.relative_motion.std_dev << 0.2, 0.2, 0.04, 0.05, 0.05, 0.05;
                args.noise.orientation.std_dev = 0.05;
                break;
            default:
                std::cout << "Unexpected argument for noise level: " << argv[1] << '\n';
                printBadArgsAndExit();
        }
        args.noise.relative_motion.mean = Eigen::Matrix<double, 6, 1>::Zero();
        args.noise.orientation.mean = 0.0;

        Eigen::Vector3d drift_p;
        Eigen::Quaterniond drift_q;
        switch (std::stoi(argv[2])) {

            case 0:
                args.drift = simulator::Drift();
                break;
            case 1:
                drift_p = Eigen::Vector3d(0.05, 0.05, 0.0);
                drift_q = Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()));
                args.drift = simulator::Drift(drift_p, drift_q);
                break;
            case 2:
                drift_p = Eigen::Vector3d(0.05, 0.05, 0.01);
                drift_q = Eigen::Quaterniond(Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX())
                                            * Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitY())
                                            * Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitZ()));
                args.drift = simulator::Drift(drift_p, drift_q);
                break;
            case 3:
                drift_p = Eigen::Vector3d(0.2, 0.2, 0.05);
                drift_q = Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX())
                                             * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())
                                             * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
                args.drift = simulator::Drift(drift_p, drift_q);
                break;
            default:
                std::cout << "Unexpected argument for drift level: " << argv[2] << '\n';
                printBadArgsAndExit();
        }

        switch (std::stoi(argv[3])) {
            case 0:
                args.loopclosure_level = Args::LoopClosureLevel::NONE;
                break;
            case 1:
                args.loopclosure_level = Args::LoopClosureLevel::SINGLE;
                break;
            case 2:
                args.loopclosure_level = Args::LoopClosureLevel::MANY;
                break;
            default:
                std::cout << "Unexpected argument for loop closure level: " << argv[3] << '\n';
                printBadArgsAndExit();
        }
        switch (std::stoi(argv[4])) {
            case 0:
                args.include_orientation_edges = false;
                args.include_gravity_edges = false;
                break;
            case 1:
                args.include_orientation_edges = true;
                args.include_gravity_edges = false;
                break;
            case 2:
                args.include_orientation_edges = false;
                args.include_gravity_edges = true;
                break;
            default:
                std::cout << "Unexpected argument for include orientation edge option: " << argv[4] << '\n';
                printBadArgsAndExit();
        }
        switch (std::stoi(argv[5])) {
            case 0:
                args.absolute_position_level = Args::AbsolutePositionLevel::NONE;
                break;
            case 1:
                args.absolute_position_level = Args::AbsolutePositionLevel::SINGLE;
                break;
            case 2:
                args.absolute_position_level = Args::AbsolutePositionLevel::TWO;
                break;
            default:
                std::cout << "Unexpected argument for absolute positions option: " << argv[5] << '\n';
                printBadArgsAndExit();
        }
    }
    catch (...) {
        printBadArgsAndExit();
    }

    return args;
}