#ifndef PIPER_FORWARD_KINEMATICS_HPP
#define PIPER_FORWARD_KINEMATICS_HPP

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>

class PiperForwardKinematics {
public:
    enum DHType { STANDARD, MODIFIED };
    
    // Constructor
    PiperForwardKinematics(DHType type = STANDARD);
    
    // Métodos públicos
    Eigen::Matrix4d computeFK(const std::vector<double>& joint_values);
    const std::vector<std::vector<double>>& getDHParams() const;
    DHType getDHType() const;
    Eigen::Matrix4d computeSingleTransform(size_t joint_index, double joint_value) const;

private:
    DHType dh_type_;
    std::vector<std::vector<double>> dh_params_;

    // Métodos privados
    void setupDHParameters();
    Eigen::Matrix4d computeTransform(double alpha, double a, double d, double theta) const;
};

#endif