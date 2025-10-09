#include "piper_kinematics/piper_analytical_ik.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

PiperAnalyticalIK::PiperAnalyticalIK(PiperForwardKinematics::DHType type) 
    : fk_(type), dh_type_(type) {
    // Initialize joint limits with default values
    joint_limits_ = {
        {-M_PI, M_PI},      // Joint 1
        {-M_PI, M_PI},      // Joint 2
        {-M_PI, M_PI},      // Joint 3
        {-M_PI, M_PI},      // Joint 4
        {-M_PI, M_PI},      // Joint 5
        {-M_PI, M_PI}       // Joint 6
    };
    
    // Extract DH parameters for easier access
    const auto& dh_params = fk_.getDHParams();
    if (dh_params.size() < 6) {
        throw std::runtime_error("Invalid DH parameters size");
    }
    
    // Store DH parameters
    a1_ = dh_params[0][1]; d1_ = dh_params[0][2]; alpha1_ = dh_params[0][0]; theta_offset1_ = dh_params[0][3];
    a2_ = dh_params[1][1]; d2_ = dh_params[1][2]; alpha2_ = dh_params[1][0]; theta_offset2_ = dh_params[1][3];
    a3_ = dh_params[2][1]; d3_ = dh_params[2][2]; alpha3_ = dh_params[2][0]; theta_offset3_ = dh_params[2][3];
    a4_ = dh_params[3][1]; d4_ = dh_params[3][2]; alpha4_ = dh_params[3][0]; theta_offset4_ = dh_params[3][3];
    a5_ = dh_params[4][1]; d5_ = dh_params[4][2]; alpha5_ = dh_params[4][0]; theta_offset5_ = dh_params[4][3];
    a6_ = dh_params[5][1]; d6_ = dh_params[5][2]; alpha6_ = dh_params[5][0]; theta_offset6_ = dh_params[5][3];

    // Debug output
    std::cout << "Analytical IK initialized with " << (dh_type_ == PiperForwardKinematics::MODIFIED ? "Modified" : "Standard") << " DH parameters:" << std::endl;
    std::cout << "a: [" << a1_ << ", " << a2_ << ", " << a3_ << ", " << a4_ << ", " << a5_ << ", " << a6_ << "]" << std::endl;
    std::cout << "d: [" << d1_ << ", " << d2_ << ", " << d3_ << ", " << d4_ << ", " << d5_ << ", " << d6_ << "]" << std::endl;
    std::cout << "alpha: [" << alpha1_ << ", " << alpha2_ << ", " << alpha3_ << ", " << alpha4_ << ", " << alpha5_ << ", " << alpha6_ << "]" << std::endl;
    std::cout << "theta_offsets: [" << theta_offset1_ << ", " << theta_offset2_ << ", " << theta_offset3_ << ", " 
              << theta_offset4_ << ", " << theta_offset5_ << ", " << theta_offset6_ << "]" << std::endl;
}

void PiperAnalyticalIK::setJointLimits(const std::vector<std::pair<double, double>>& limits) {
    if (limits.size() != 6) {
        throw std::runtime_error("Joint limits must be specified for all 6 joints");
    }
    joint_limits_ = limits;
}

const std::vector<std::pair<double, double>>& PiperAnalyticalIK::getJointLimits() const {
    return joint_limits_;
}

std::vector<std::vector<double>> PiperAnalyticalIK::computeIK(const Eigen::Matrix4d& target_pose, 
                                                            bool filter_by_limits,
                                                            bool verbose) {
    std::vector<std::vector<double>> all_solutions;
    
    try {
        // Step 1: Extract position and orientation from target pose
        Eigen::Vector3d p_target = target_pose.block<3,1>(0,3);
        Eigen::Matrix3d R_target = target_pose.block<3,3>(0,0);
        
        if (verbose) {
            std::cout << "Target position: " << p_target.transpose() << std::endl;
            std::cout << "Target orientation:\n" << R_target << std::endl;
        }
        
        // Step 2: Calculate wrist center position
        Eigen::Vector3d wrist_center = calculateWristCenter(p_target, R_target);
        
        if (verbose) {
            std::cout << "Wrist center: " << wrist_center.transpose() << std::endl;
        }
        
        // Step 3: Solve for joints 1, 2, 3 (position part)
        std::vector<std::array<double, 3>> arm_solutions = solveArmJoints(wrist_center);
        
        if (verbose) {
            std::cout << "Found " << arm_solutions.size() << " arm solutions" << std::endl;
        }
        
        // Step 4: For each arm solution, solve for joints 4, 5, 6 (orientation part)
        for (const auto& arm_sol : arm_solutions) {
            if (verbose) {
                std::cout << "Arm solution: [" << arm_sol[0] << ", " << arm_sol[1] << ", " << arm_sol[2] << "]" << std::endl;
            }
            std::vector<std::array<double, 3>> wrist_solutions = solveWristJoints(arm_sol, R_target);
            
            if (verbose) {
                std::cout << "Found " << wrist_solutions.size() << " wrist solutions" << std::endl;
            }
            
            for (const auto& wrist_sol : wrist_solutions) {
                if (verbose) {
                    std::cout << "Wrist solution: [" << wrist_sol[0] << ", " << wrist_sol[1] << ", " << wrist_sol[2] << "]" << std::endl;
                }
                
                // Combine arm and wrist solutions
                std::vector<double> full_solution = {
                    arm_sol[0], arm_sol[1], arm_sol[2],
                    wrist_sol[0], wrist_sol[1], wrist_sol[2]
                };
                    
                // Apply joint offsets
                applyJointOffsets(full_solution);
                
                // Normalize angles to [-π, π]
                normalizeJointAngles(full_solution);
                
                // Check joint limits if requested
                if (!filter_by_limits || withinJointLimits(full_solution)) {
                    all_solutions.push_back(full_solution);
                }
            }
        }
        
        if (verbose) {
            std::cout << "Total solutions found: " << all_solutions.size() << std::endl;
        }
        
    } catch (const std::exception& e) {
        if (verbose) {
            std::cerr << "Analytical IK failed: " << e.what() << std::endl;
        }
        throw;
    }
    
    return all_solutions;
}

std::vector<double> PiperAnalyticalIK::findBestSolution(const Eigen::Matrix4d& target_pose,
                                                      const std::vector<double>& current_joints,
                                                      bool verbose) {
    // Get all possible solutions
    auto all_solutions = computeIK(target_pose, true, verbose);
    
    if (all_solutions.empty()) {
        throw std::runtime_error("No valid IK solutions found");
    }
    
    // Create subset of first 6 joints
    std::vector<double> current_subset(current_joints.begin(), current_joints.begin() + 6);

    // Find solution closest to current joints
    double min_distance = std::numeric_limits<double>::max();
    std::vector<double> best_solution;
    
    for (const auto& solution : all_solutions) {
        double distance = calculateJointDistance(solution, current_subset);
        if (distance < min_distance) {
            min_distance = distance;
            best_solution = solution;
        }
    }
    
    return best_solution;
}

Eigen::Vector3d PiperAnalyticalIK::calculateWristCenter(const Eigen::Vector3d& p_target, const Eigen::Matrix3d& R_target) {
    Eigen::Vector3d z_6 = R_target.col(2);
    if (dh_type_ == PiperForwardKinematics::STANDARD) {


        return p_target - d6_ * z_6;
    } else {


        return p_target - d6_ * z_6;
    }
}

std::vector<std::array<double, 3>> PiperAnalyticalIK::solveArmJoints(const Eigen::Vector3d& wrist_center) {
    std::vector<std::array<double, 3>> solutions;
    
    double x = wrist_center(0);
    double y = wrist_center(1);
    double z = wrist_center(2);
    
    
    if (dh_type_ == PiperForwardKinematics::STANDARD) {
        z -= d1_;  
    } else {
        z -= d1_;  
    }
    
    double r = std::sqrt(x*x + y*y);
    
    double L2 = a2_;  // 0.28503
    double L3 = std::sqrt(a3_*a3_ + d4_*d4_);  // sqrt((-0.021984)^2 + 0.25075^2)
    
    double D = (r*r + z*z - L2*L2 - L3*L3) / (2.0 * L2 * L3);
    
    if (std::abs(D) > 1.0 + 1e-6) {
        std::cout << "No solution: D = " << D << " out of range [-1, 1]" << std::endl;
        return solutions;
    }
    
    double beta = std::acos(std::clamp(D, -1.0, 1.0));
    double phi = std::atan2(d4_, std::abs(a3_));  // 注意a3_为负值
    

    for (double sign : {1.0, -1.0}) {
        double q3 = sign * beta - phi;
        
        // 计算q2
        double k1 = L2 + L3 * std::cos(q3 + phi);
        double k2 = L3 * std::sin(q3 + phi);
        
        double gamma = std::atan2(z, r);
        double delta = std::atan2(k2, k1);
        
        double q2 = gamma - delta;
        
        // 计算q1
        double q1 = std::atan2(y, x);
        
        solutions.push_back({q1, q2, q3});
    }
    
    return solutions;
}

std::vector<std::array<double, 3>> PiperAnalyticalIK::solveWristJoints(const std::array<double, 3>& arm_joints,
                                                                     const Eigen::Matrix3d& R_target) {
    std::vector<std::array<double, 3>> solutions;
    const double q1 = arm_joints[0];
    const double q2 = arm_joints[1];
    const double q3 = arm_joints[2];


    Eigen::Matrix3d R03 = computeR03(q1, q2, q3);
    
    Eigen::Matrix3d R36 = R03.transpose() * R_target;

    const double r11 = R36(0, 0), r12 = R36(0, 1), r13 = R36(0, 2);
    const double r21 = R36(1, 0), r22 = R36(1, 1), r23 = R36(1, 2);
    const double r31 = R36(2, 0), r32 = R36(2, 1), r33 = R36(2, 2);

    for (double sign : {1.0, -1.0}) {
        double q5;
        
        
        if (std::abs(r33) > 0.9999) {

            q5 = (r33 > 0) ? 0.0 : M_PI;
            double q4 = 0.0; 
            double q6 = std::atan2(r21, r11);
            solutions.push_back({q4, q5, q6});
        } else {

            q5 = sign * std::acos(r33);
            double q4 = std::atan2(r23, r13);
            double q6 = std::atan2(r32, -r31);
            
            if (sign < 0) {
                q4 += M_PI;
                q6 -= M_PI;
            }
            
            q4 = std::atan2(std::sin(q4), std::cos(q4));
            q6 = std::atan2(std::sin(q6), std::cos(q6));
            
            solutions.push_back({q4, q5, q6});
        }
    }
    return solutions;
}

Eigen::Matrix3d PiperAnalyticalIK::computeR03(double theta1, double theta2, double theta3) {

    Eigen::Matrix4d T01, T12, T23;
    
    if (dh_type_ == PiperForwardKinematics::MODIFIED) {

        T01 = modifiedDHTransform(alpha1_, a1_, d1_, theta1 + theta_offset1_);
        T12 = modifiedDHTransform(alpha2_, a2_, d2_, theta2 + theta_offset2_);
        T23 = modifiedDHTransform(alpha3_, a3_, d3_, theta3 + theta_offset3_);
    } else {

        T01 = standardDHTransform(alpha1_, a1_, d1_, theta1 + theta_offset1_);
        T12 = standardDHTransform(alpha2_, a2_, d2_, theta2 + theta_offset2_);
        T23 = standardDHTransform(alpha3_, a3_, d3_, theta3 + theta_offset3_);
    }
    
    Eigen::Matrix4d T03 = T01 * T12 * T23;
    return T03.block<3,3>(0,0); 
}

Eigen::Matrix4d PiperAnalyticalIK::modifiedDHTransform(double alpha, double a, double d, double theta) {
    Eigen::Matrix4d T;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    
    T << ct, -st, 0, a,
         st*ca, ct*ca, -sa, -sa*d,
         st*sa, ct*sa, ca, ca*d,
         0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d PiperAnalyticalIK::standardDHTransform(double alpha, double a, double d, double theta) {
    Eigen::Matrix4d T;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    
    T << ct, -st*ca, st*sa, a*ct,
         st, ct*ca, -ct*sa, a*st,
         0, sa, ca, d,
         0, 0, 0, 1;
    return T;
}

void PiperAnalyticalIK::applyJointOffsets(std::vector<double>& joint_values) {
    if (joint_values.size() >= 1) joint_values[0] -= theta_offset1_;
    if (joint_values.size() >= 2) joint_values[1] -= theta_offset2_;
    if (joint_values.size() >= 3) joint_values[2] -= theta_offset3_;
    if (joint_values.size() >= 4) joint_values[3] -= theta_offset4_;
    if (joint_values.size() >= 5) joint_values[4] -= theta_offset5_;
    if (joint_values.size() >= 6) joint_values[5] -= theta_offset6_;
}

void PiperAnalyticalIK::normalizeJointAngles(std::vector<double>& joints) {
    for (auto& angle : joints) {
        // 使用更稳健的规范化方法
        angle = std::fmod(angle, 2.0 * M_PI);
        if (angle > M_PI) angle -= 2.0 * M_PI;
        if (angle < -M_PI) angle += 2.0 * M_PI;
    }
}

bool PiperAnalyticalIK::withinJointLimits(const std::vector<double>& joint_values) {
    if (joint_values.size() != joint_limits_.size()) {
        std::cout << "Joint size mismatch" << std::endl;
        return false;
    }
    
    for (size_t i = 0; i < joint_values.size(); ++i) {
        if (joint_values[i] < joint_limits_[i].first || joint_values[i] > joint_limits_[i].second) {
            std::cout << "Joint " << i+1 << " out of limits: " << joint_values[i] 
                    << " not in [" << joint_limits_[i].first << ", " << joint_limits_[i].second << "]" << std::endl;
            return false;
        }
    }
    return true;
}

double PiperAnalyticalIK::calculateJointDistance(const std::vector<double>& joints1,
                                               const std::vector<double>& joints2) {
    if (joints1.size() != joints2.size()) {
        return std::numeric_limits<double>::max();
    }
    
    double distance = 0.0;
    for (size_t i = 0; i < joints1.size(); ++i) {
        double diff = joints1[i] - joints2[i];
        // Handle angle wrap-around
        if (diff > M_PI) diff -= 2.0 * M_PI;
        if (diff < -M_PI) diff += 2.0 * M_PI;
        distance += diff * diff;
    }
    
    return std::sqrt(distance);
}