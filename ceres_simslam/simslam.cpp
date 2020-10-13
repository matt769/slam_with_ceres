//
// Created by matt on 04/10/2020.
//

#include <vector>
#include <iostream>
#include <fstream>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "cost_functions.h"
#include "graph.h"
#include "pose.h"

// start off simple, following pose_graph_3d but with my own generated data
// then try using eigen structures
// then maybe simulate the robot a bit more 'freely'?

RelativeMotion addNoise(const RelativeMotion& motion, const double p_noise_stdev, std::default_random_engine& generator) {
    // TODO add rotation noise
    // generate some 'random' noise
    std::normal_distribution<double> distribution(0.0, p_noise_stdev);
    RelativeMotion noisy_motion(motion);
    noisy_motion.p_.x() = distribution(generator);
    noisy_motion.p_.y() = distribution(generator);
//    noisy_motion.p_.z() = distribution(generator);
    noisy_motion.p_.z() = motion.p_.z();
    return noisy_motion;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    std::default_random_engine noise_generator(0);

    std::vector<Pose> true_trajectory;
    std::vector<Edge> edges;
    std::vector<Node> nodes;
    std::vector<Edge> noisy_edges;
    std::vector<Node> noisy_nodes;

    // just move around in xy plane, no rotation
    const Eigen::Vector3d forward(1.0, 0.0, 0.0);
    const Eigen::Vector3d backward = -forward;
    const Eigen::Vector3d left(0.0, 1.0, 0.0);
    const Eigen::Vector3d right = -left;

    // add starting node
    // TODO review all this - have better support for noisy and ground truth versions
    size_t node_id = 0;
    size_t noisy_node_id = 0;
    nodes.emplace_back(Node{node_id++, Pose()});
    noisy_nodes.emplace_back(Node{node_id++, Pose()});

    auto buildGraph = [&](const RelativeMotion& motion) {
        Pose new_pose = nodes.back().pose_ * motion;
        nodes.emplace_back(Node{node_id++, new_pose});
        edges.emplace_back(Edge{node_id-2, node_id-1, motion}); // URGH

        RelativeMotion noisy_motion = addNoise(motion, 0.05, noise_generator);
        Pose new_noisy_pose = nodes.back().pose_ * noisy_motion;
        noisy_nodes.emplace_back(Node{noisy_node_id++, new_noisy_pose});
        noisy_edges.emplace_back(Edge{noisy_node_id-2, noisy_node_id-1, noisy_motion}); // URGH
    };

    constexpr size_t steps_fw_bw = 4;
    constexpr size_t steps_left_right = 3;
    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        RelativeMotion motion(forward, Eigen::Quaterniond::Identity());
        buildGraph(motion);
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        RelativeMotion motion(left, Eigen::Quaterniond::Identity());
        buildGraph(motion);
    }
    for (size_t idx = 0; idx < steps_fw_bw; ++idx) {
        RelativeMotion motion(backward, Eigen::Quaterniond::Identity());
        buildGraph(motion);
    }
    for (size_t idx = 0; idx < steps_left_right; ++idx) {
        RelativeMotion motion(right, Eigen::Quaterniond::Identity());
        buildGraph(motion);
    }

    // create a loop closure at the end
    RelativeMotion T_end_start = nodes.back().pose_.inverse() * nodes.front().pose_;
    edges.emplace_back(Edge{node_id-1, 0, T_end_start});
    RelativeMotion T_end_start_noisy = addNoise(T_end_start, 0.05, noise_generator);
    noisy_edges.emplace_back(Edge{noisy_node_id-1, 0, T_end_start_noisy});



    // TODO tidy up data structures
    //  maybe add graph structure
    //  and make node id generation better

    for (const auto& node: nodes) {
        std::cout << node.id_ << '\t' << node.pose_.p_.transpose() << '\n';
    }
    std::cout << '\n';
    for (const auto& edge: edges) {
        std::cout << edge.start << '\t' << edge.end << '\t' << edge.relative_motion.p_.transpose() << '\n';
    }
    std::cout << '\n';
//    std::cout << T_end_start.p_.transpose() << '\n';

    // TODO add some noise to the odometry
    //  and make sure the nodes don't start at the exact correct position!

    LOG(INFO) << nodes.size() << '\t' << edges.size();
    CHECK(nodes.size() == edges.size());
    CHECK(nodes.size() == noisy_nodes.size());
    CHECK(edges.size() == noisy_edges.size());

    // output graph current state
    std::ofstream output_file;
    output_file.open("true_poses.txt");
    for (const auto& node: nodes) {
        output_file << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }

    std::ofstream output_file_noisy;
    output_file_noisy.open("noisy_poses.txt");
    for (const auto& node: noisy_nodes) {
        output_file_noisy << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }

    // create Ceres problem and optimise
    ceres::Problem problem;
    ceres::LossFunction* loss_function = nullptr;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    // TODO maybe I should make a better data structure for my graph?




    for (const auto& edge: noisy_edges) {
        ceres::CostFunction* cost_function = RelativeMotionCost::Create(edge.relative_motion);
        problem.AddResidualBlock(cost_function, loss_function,
                                noisy_nodes[edge.start].pose_.p_.data(), noisy_nodes[edge.start].pose_.q_.coeffs().data(),
                                 noisy_nodes[edge.end].pose_.p_.data(), noisy_nodes[edge.end].pose_.q_.coeffs().data());
        problem.SetParameterization(noisy_nodes[edge.start].pose_.q_.coeffs().data(),
                                     quaternion_local_parameterization);
        problem.SetParameterization(noisy_nodes[edge.end].pose_.q_.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    problem.SetParameterBlockConstant(noisy_nodes[0].pose_.p_.data());
    problem.SetParameterBlockConstant(noisy_nodes[0].pose_.q_.coeffs().data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << '\n';
    // TODO output before and after

    // output graph optimised state
    std::ofstream output_file_opt;
    output_file_opt.open("optimised_poses.txt");
    for (const auto& node: noisy_nodes) {
        output_file_opt << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }

    // compare true and optimised

    // total trajectory distance (simple metric)



    return 0;
}

// TODO find out why my static function for generating node ids caused a compilation error
//  (when filling vector?)