#include "piper_kinematics/piper_forward_kinematics.hpp"
#include <stdexcept>

PiperForwardKinematics::PiperForwardKinematics(DHType type) : dh_type_(type) {
    setupDHParameters();
}

void PiperForwardKinematics::setupDHParameters() {
    if (dh_type_ == STANDARD) {
        // 标准DH参数 [alpha, a, d, theta_offset]
        dh_params_ = {
            {-M_PI/2,   0,          0.123,      0},                     // Joint 1
            {0,         0.28503,    0,          -172.22/180*M_PI},      // Joint 2 
            {M_PI/2,    -0.021984,  0,          -102.78/180*M_PI},      // Joint 3
            {-M_PI/2,   0,          0.25075,    0},                     // Joint 4
            {M_PI/2,    0,          0,          0},                     // Joint 5
            {0,         0,          0.211,      0}                      // Joint 6
        };
    } else {
        // 改进DH参数 [alpha, a, d, theta_offset]
        dh_params_ = {
            {0,         0,          0.123,      0},                     // Joint 1
            {-M_PI/2,   0,          0,          -172.22/180*M_PI},      // Joint 2 
            {0,         0.28503,    0,          -102.78/180*M_PI},      // Joint 3
            {M_PI/2,    -0.021984,  0.25075,    0},                     // Joint 4
            {-M_PI/2,   0,          0,          0},                     // Joint 5
            {M_PI/2,    0,          0.211,      0}                      // Joint 6
        };
    }
}

Eigen::Matrix4d PiperForwardKinematics::computeFK(const std::vector<double>& joint_values) {
    if (joint_values.size() < 6) {
        throw std::runtime_error("Piper arm requires at least 6 joint values for FK");
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (size_t i = 0; i < 6; ++i) {
        double theta = joint_values[i] + dh_params_[i][3];  // θ = joint_value + θ_offset
        double d = dh_params_[i][2];                       // d = d_fixed
        
        T = T * computeTransform(
            dh_params_[i][0],  // alpha
            dh_params_[i][1],  // a
            d,                 // d
            theta              // theta
        );
    }
    return T;
}

Eigen::Matrix4d PiperForwardKinematics::computeSingleTransform(size_t joint_index, double joint_value) const {
    if (joint_index >= dh_params_.size()) {
        throw std::runtime_error("Joint index out of range");
    }

    double theta = joint_value + dh_params_[joint_index][3];
    double d = dh_params_[joint_index][2];
    
    return computeTransform(
        dh_params_[joint_index][0],  // alpha
        dh_params_[joint_index][1],  // a
        d,                          // d
        theta                       // theta
    );
}

Eigen::Matrix4d PiperForwardKinematics::computeTransform(double alpha, double a, double d, double theta) const {
    Eigen::Matrix4d T;
    if (dh_type_ == STANDARD) {
        // 标准DH变换矩阵
        double ct = std::cos(theta), st = std::sin(theta);
        double ca = std::cos(alpha), sa = std::sin(alpha);
        
        T << ct, -st*ca,  st*sa, a*ct,
             st,  ct*ca, -ct*sa, a*st,
             0,   sa,     ca,    d,
             0,   0,      0,     1;
    } else {
        // 改进DH变换矩阵
        double ct = std::cos(theta), st = std::sin(theta);
        double ca = std::cos(alpha), sa = std::sin(alpha);
        
        T << ct,            -st,            0,             a,
             st*ca,  ct*ca, -sa, -sa*d,
             st*sa,  ct*sa,  ca,  ca*d,
             0,      0,      0,   1;
    }
    return T;
}

const std::vector<std::vector<double>>& PiperForwardKinematics::getDHParams() const {
    return dh_params_;
}

PiperForwardKinematics::DHType PiperForwardKinematics::getDHType() const {
    return dh_type_;
}