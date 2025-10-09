#ifndef PIPER_ANALYTICAL_IK_HPP
#define PIPER_ANALYTICAL_IK_HPP

#pragma once
#include "piper_forward_kinematics.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <limits>
#include <array>

class PiperAnalyticalIK {
public:
    PiperAnalyticalIK(PiperForwardKinematics::DHType type = PiperForwardKinematics::MODIFIED);
    
    // Set joint limits (min, max) for each joint
    void setJointLimits(const std::vector<std::pair<double, double>>& limits);
    
    // Get joint limits
    const std::vector<std::pair<double, double>>& getJointLimits() const;

    // Main analytical IK solver
    std::vector<std::vector<double>> computeIK(const Eigen::Matrix4d& target_pose, 
                                              bool filter_by_limits = true,
                                              bool verbose = false);

    // Find the best solution based on proximity to current joints
    std::vector<double> findBestSolution(const Eigen::Matrix4d& target_pose,
                                        const std::vector<double>& current_joints,
                                        bool verbose = false);

private:
    PiperForwardKinematics fk_;
    PiperForwardKinematics::DHType dh_type_;
    std::vector<std::pair<double, double>> joint_limits_;
    
    // DH parameters stored for easy access
    double a1_, a2_, a3_, a4_, a5_, a6_;
    double d1_, d2_, d3_, d4_, d5_, d6_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_, alpha6_;
    double theta_offset1_, theta_offset2_, theta_offset3_, theta_offset4_, theta_offset5_, theta_offset6_;

    // Private methods
    Eigen::Vector3d calculateWristCenter(const Eigen::Vector3d& p_target, const Eigen::Matrix3d& R_target);
    std::vector<std::array<double, 3>> solveArmJoints(const Eigen::Vector3d& wrist_center);
    std::vector<std::array<double, 3>> solveWristJoints(const std::array<double, 3>& arm_joints,
                                                      const Eigen::Matrix3d& R_target);
    Eigen::Matrix3d computeR03(double theta1, double theta2, double theta3);
    Eigen::Matrix4d modifiedDHTransform(double alpha, double a, double d, double theta);
    Eigen::Matrix4d standardDHTransform(double alpha, double a, double d, double theta);
    void applyJointOffsets(std::vector<double>& joint_values);
    void normalizeJointAngles(std::vector<double>& joints);
    bool withinJointLimits(const std::vector<double>& joint_values);
    double calculateJointDistance(const std::vector<double>& joints1,
                                 const std::vector<double>& joints2);
};

#endif // PIPER_ANALYTICAL_IK_HPP