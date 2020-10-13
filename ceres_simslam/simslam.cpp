//
// Created by matt on 04/10/2020.
//

#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <fstream>

#include "cost_functions.h"
#include "pose.h"

// start off simple, following pose_graph_3d but with my own generated data
// then try using eigen structures
// then maybe simulate the robot a bit more 'freely'?

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


int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);

    std::vector<Pose> true_trajectory;
    std::vector<Edge> edges;
    std::vector<Node> nodes;

    // just move around in xy plane, no rotation
    const Eigen::Vector3d forward(1.0, 0.0, 0.0);
    const Eigen::Vector3d backward = -forward;
    const Eigen::Vector3d left(0.0, 1.0, 0.0);
    const Eigen::Vector3d right = -left;

    // add starting node
    size_t node_id = 0;
    nodes.emplace_back(Node{node_id++, Pose()});

//    auto buildGraph = [&](const RelativeMotion& motion) {
//        Node start_node = nodes.back();
//        Pose new_pose = start_node.pose_ * motion;
//        nodes.emplace_back(new_pose);
//        size_t end_id = nodes.back().id_;
//        edges.emplace_back(Edge{start_node.id_, end_id, motion});
//    };

    auto buildGraph = [&](const RelativeMotion& motion) {
        Pose new_pose = nodes.back().pose_ * motion;
        nodes.emplace_back(Node{node_id++, new_pose});
        edges.emplace_back(Edge{node_id-2, node_id-1, motion}); // URGH
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

    // populate nodes and edges structures
    // TODO tidy up these duplicated data structures and their creation/population later

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

    // output graph current state
    std::ofstream output_file;
    output_file.open("initial_poses.txt");
    for (const auto& node: nodes) {
        output_file << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }

    // create Ceres problem and optimise
    ceres::Problem problem;
    ceres::LossFunction* loss_function = nullptr;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    // TODO maybe I should make a better data structure for my graph?




    for (const auto& edge: edges) {
        ceres::CostFunction* cost_function = RelativeMotionCost::Create(edge.relative_motion);
        problem.AddResidualBlock(cost_function, loss_function,
                                nodes[edge.start].pose_.p_.data(), nodes[edge.start].pose_.q_.coeffs().data(),
                                nodes[edge.end].pose_.p_.data(), nodes[edge.end].pose_.q_.coeffs().data());
        problem.SetParameterization(nodes[edge.start].pose_.q_.coeffs().data(),
                                     quaternion_local_parameterization);
        problem.SetParameterization(nodes[edge.end].pose_.q_.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    problem.SetParameterBlockConstant(nodes[0].pose_.p_.data());
    problem.SetParameterBlockConstant(nodes[0].pose_.q_.coeffs().data());

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
    for (const auto& node: nodes) {
        output_file_opt << node.id_ << ' ' << node.pose_.p_.transpose() << ' ' << node.pose_.q_.coeffs().transpose() << '\n';
    }

    return 0;
}

// TODO find out why my static function for generating node ids caused a compilation error
//  (when filling vector?)