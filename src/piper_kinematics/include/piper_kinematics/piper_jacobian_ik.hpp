#ifndef PIPER_JACOBIAN_IK_HPP
#define PIPER_JACOBIAN_IK_HPP

#include "piper_forward_kinematics.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <algorithm>

class PiperJacobianIK {
public:
    PiperJacobianIK(PiperForwardKinematics::DHType type = PiperForwardKinematics::STANDARD);
    
    bool success = false;

    // Configuration methods
    void setMaxIterations(int max_iter);
    void setPositionTolerance(double tol);
    void setOrientationTolerance(double tol);
    void setDampingFactor(double lambda);
    void useAnalyticalJacobian(bool use);
    void setDHType(PiperForwardKinematics::DHType type);
    void setJointLimits(const std::vector<std::pair<double, double>>& limits);
    
    // Main IK solver
    std::vector<double> computeIK(const std::vector<double>& initial_guess, 
                                 const Eigen::Matrix4d& target_pose,
                                 bool verbose = false,
                                 Eigen::VectorXd* final_error = nullptr);

    // For backward compatibility
    void setTolerance(double tol);
    
    // Verification
    Eigen::VectorXd verifySolution(const std::vector<double>& joint_values, 
                                  const Eigen::Matrix4d& target_pose);

private:
    PiperForwardKinematics fk_;
    int max_iterations_;
    double position_tolerance_;
    double orientation_tolerance_;
    double lambda_;
    bool use_analytical_jacobian_;
    std::vector<std::pair<double, double>> joint_limits_;

    // Private methods
    void normalizeJointAngles(std::vector<double>& joints);
    Eigen::VectorXd computePoseError(const Eigen::Matrix4d& current, const Eigen::Matrix4d& target);
    Eigen::MatrixXd computeNumericalJacobian(const std::vector<double>& joint_values);
    Eigen::MatrixXd computeAnalyticalJacobian(const std::vector<double>& joint_values, 
                                            const Eigen::Matrix4d& current_pose);
};

#endif