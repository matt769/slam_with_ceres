//
// Created by matt on 04/10/2020.
//

#ifndef CERES_SIMSLAM_COST_FUNCTIONS_H
#define CERES_SIMSLAM_COST_FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "graph.h"
#include "pose.h"

// Needs to store the measurement upon construction
//  and then take the poses as parameters when called
// The exact form of the residuals (size, calculation etc) could be changed if desired

// Ultimately I've ended up writing out almost exactly the same cost function class as used in
//  the Ceres pose_graph_3d example, but it was useful to go through it myself
class RelativeMotionCost {
public:
    //  The observed edge is the observed position of the second pose represented in the frame of the first
    RelativeMotionCost(const graph::RelativeMotionEdge& observed_edge)
        : observed_edge_(observed_edge) {}

    // This function takes in T[3] position T[4] quaternion for each pose
    // and returns T[6] residuals
    template <typename T>
    bool operator()(const T* const a_p_ptr, const T* const a_q_ptr, const T* const b_p_ptr, const T* const b_q_ptr, T* residuals_ptr) const {
        // this object will be called with arrays - we need (want) to map these to our familiar eigen objects
        //  for easier calculations
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> a_p(a_p_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> a_q(a_q_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> b_p(b_p_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> b_q(b_q_ptr);

        // Difference the two input poses
        // Calculated in the frame of the first pose
//        Pose a(a_p, a_q);
//        Pose b(b_p, b_q); // can't use Pose as it would have to be templated (to allow ceres::Jet type)
//        RelativeMotion implied_edge = a.inverse() * b;

        // a_p - b_p is the difference in whatever shared coordinate frame the poses are given in
        //  so we need the 'a_q.conjugate() * ' to get this w.r.t. pose a
        Eigen::Matrix<T, 3, 1> p_ab = a_q.conjugate() * (b_p - a_p);
        Eigen::Quaternion<T> q_ab = a_q.conjugate() * b_q;

        // In the pose_graph_3d example, they choose the errors to be
        //  the position vector difference
        //  and double the vector part of the quaternion representing the difference in orientation
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3,1>(0,0) = observed_edge_.relative_motion.p_.template cast<T>() - p_ab; // POSE_GRAPH_3D example has these the other way around??
        residuals.template block<3, 1>(3, 0) = T(2.0) * (observed_edge_.relative_motion.q_.template cast<T>() * q_ab.conjugate()).vec();

        // Weight the residuals by uncertainty
        residuals.applyOnTheLeft(observed_edge_.sqrt_info.template cast<T>());;

        return true;
    }

    // So that we don't need to specify all the template parameters etc when we use this, follow the
    //  existing convention and create a convenience function to return us the CostFunction object (pointer) we need
    // Slightly oddly, the first non-type template parameter is the size of the output/residual array
    static ceres::CostFunction* Create(const graph::RelativeMotionEdge& observed_edge) {
        return (new ceres::AutoDiffCostFunction<RelativeMotionCost, 6, 3, 4, 3, 4>(
                new RelativeMotionCost(observed_edge)));
    }

private:
    graph::RelativeMotionEdge observed_edge_;
};


class OrientationCost {
public:
    //  The measurement is the orientation of a fixed frame (e.g. inertial) from a node
    //
    OrientationCost(const graph::OrientationEdge& measured_orientation)
            : measured_orientation_(measured_orientation) {}

    // This function takes in a T[4] quaternion for the pose of the node
    // and returns T[3] residuals
    template <typename T>
    bool operator()(const T* const q_ptr, T* residuals_ptr) const {
        // this object will be called with arrays - we need (want) to map these to our familiar eigen objects
        //  for easier calculations
        Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);

        // The measurement of the fixed frame should be consistent for all nodes,
        //  in whatever frame the nodes are described in
        // e.g. like a compass always pointing North
        // I.e. node pose * measurement = constant
        // But what's the constant?
        // If we just say that it's zero (identity rotation) and try and minimise this,
        //  then we are essentially aligning our measured fixed frame with the frame in which we describe the nodes

        Eigen::Quaternion<T> delta_q = q * measured_orientation_.orientation.cast<T>();

        // We'll create the rotation errors as for the RelativeMotionCost
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = T(2.0) * delta_q.vec();

        // Weight the residuals by uncertainty
        residuals.applyOnTheLeft(measured_orientation_.sqrt_info.template cast<T>());;

        return true;
    }

    static ceres::CostFunction* Create(const graph::OrientationEdge& measured_orientation) {
        return (new ceres::AutoDiffCostFunction<OrientationCost, 3, 4>(
                new OrientationCost(measured_orientation)));
    }

private:
    graph::OrientationEdge measured_orientation_;
};


class GravityCost {
public:
    //  The measurement is the direction of gravity w.r.t. a node
    GravityCost(const graph::GravityEdge& measurement)
            : measurement_(measurement) {}

    // This function takes in a T[4] quaternion for the pose of the node
    // and returns T[3] residuals
    template <typename T>
    bool operator()(const T* const q_ptr, T* residuals_ptr) const {
        Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);

        // If we say that the direction of gravity should be along -Z
        //  and we try to minimise the different between that and the measurement
        //  then we essentially are aligning the frame in which we describe the nodes with a frame that has
        //  it's horizontal plane orthogonal to gravity e.g. a local Earth-fixed coordinate frame

        Eigen::Matrix<T, 3, 1> gravity_vector = q * measurement_.gravity_vector.cast<T>();
        // we want a node orientation that produces a gravity measurement equal to 0,0,-1
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = gravity_vector + Eigen::Matrix<T, 3, 1>::UnitZ();

        // Weight the residuals by uncertainty
        residuals.applyOnTheLeft(measurement_.sqrt_info.template cast<T>());;

        return true;
    }

    static ceres::CostFunction* Create(const graph::GravityEdge& measurement) {
        return (new ceres::AutoDiffCostFunction<GravityCost, 3, 4>(
                new GravityCost(measurement)));
    }

private:
    graph::GravityEdge measurement_;
};


#endif //CERES_SIMSLAM_COST_FUNCTIONS_H
