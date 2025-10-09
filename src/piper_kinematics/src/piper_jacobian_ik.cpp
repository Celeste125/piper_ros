#include "piper_kinematics/piper_jacobian_ik.hpp"
#include <cmath>
#include <stdexcept>

PiperJacobianIK::PiperJacobianIK(PiperForwardKinematics::DHType type) 
    : fk_(type), max_iterations_(100), position_tolerance_(1e-5), 
      orientation_tolerance_(1e-3), lambda_(0.1), use_analytical_jacobian_(false) {
    // Initialize with no joint limits by default
    joint_limits_.resize(6, std::make_pair(-M_PI, M_PI));
}

void PiperJacobianIK::setMaxIterations(int max_iter) { 
    max_iterations_ = max_iter; 
}

void PiperJacobianIK::setPositionTolerance(double tol) { 
    position_tolerance_ = tol; 
}

void PiperJacobianIK::setOrientationTolerance(double tol) { 
    orientation_tolerance_ = tol; 
}

void PiperJacobianIK::setDampingFactor(double lambda) { 
    lambda_ = lambda; 
}

void PiperJacobianIK::useAnalyticalJacobian(bool use) { 
    use_analytical_jacobian_ = use; 
}

void PiperJacobianIK::setDHType(PiperForwardKinematics::DHType type) { 
    fk_ = PiperForwardKinematics(type); 
}

void PiperJacobianIK::setJointLimits(const std::vector<std::pair<double, double>>& limits) {
    if (limits.size() != 6) {
        throw std::runtime_error("Joint limits must be specified for all 6 joints");
    }
    joint_limits_ = limits;
}

std::vector<double> PiperJacobianIK::computeIK(const std::vector<double>& initial_guess, 
                                             const Eigen::Matrix4d& target_pose,
                                             bool verbose,
                                             Eigen::VectorXd* final_error) {
    if (initial_guess.size() < 6) {
        throw std::runtime_error("Initial guess must have at least 6 joint values");
    }

    std::vector<double> joint_values = initial_guess;
    Eigen::Matrix4d current_pose;
    Eigen::VectorXd error(6);
    bool local_success = false;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        current_pose = fk_.computeFK(joint_values);
        error = computePoseError(current_pose, target_pose);

        if (verbose) {
            std::cout << "Iteration " << iter << ": error norm = " << error.norm() 
                      << " (pos: " << error.head<3>().norm() 
                      << ", orient: " << error.tail<3>().norm() << ")\n";
        }

        // Check convergence
        if (error.head<3>().norm() < position_tolerance_ && 
            error.tail<3>().norm() < orientation_tolerance_) {
            local_success = true;
            break;
        }

        // Compute Jacobian
        Eigen::MatrixXd J = use_analytical_jacobian_ ? 
            computeAnalyticalJacobian(joint_values, current_pose) :
            computeNumericalJacobian(joint_values);

        // Damped least squares (Levenberg-Marquardt)
        Eigen::MatrixXd Jt = J.transpose();
        Eigen::MatrixXd JJt = J * Jt;
        JJt.diagonal().array() += lambda_ * lambda_;
        
        try {
            Eigen::VectorXd delta_theta = Jt * JJt.ldlt().solve(error);

            // Update joint values with clamping to limits
            for (int i = 0; i < 6; ++i) {
                double new_value = joint_values[i] + delta_theta(i);
                // Clamp to joint limits
                joint_values[i] = std::clamp(new_value, joint_limits_[i].first, joint_limits_[i].second);
            }

            // Normalize angles to [-π, π]
            normalizeJointAngles(joint_values);
        } catch (const std::exception& e) {
            if (verbose) {
                std::cout << "Matrix solve failed: " << e.what() << std::endl;
            }
            break;
        }
    }

    if (!local_success) {
        success = false;
        throw std::runtime_error("IK did not converge within maximum iterations");
    }

    success = true;

    // Compute final error if requested
    if (final_error != nullptr) {
        current_pose = fk_.computeFK(joint_values);
        *final_error = computePoseError(current_pose, target_pose);
    }

    return joint_values;
}

void PiperJacobianIK::setTolerance(double tol) { 
    setPositionTolerance(tol);
    setOrientationTolerance(tol*10);
}

Eigen::VectorXd PiperJacobianIK::verifySolution(const std::vector<double>& joint_values, 
                                              const Eigen::Matrix4d& target_pose) {
    Eigen::Matrix4d current_pose = fk_.computeFK(joint_values);
    return computePoseError(current_pose, target_pose);
}

void PiperJacobianIK::normalizeJointAngles(std::vector<double>& joints) {
    for (auto& angle : joints) {
        angle = std::fmod(angle + M_PI, 2*M_PI) - M_PI;
    }
}

Eigen::VectorXd PiperJacobianIK::computePoseError(const Eigen::Matrix4d& current, const Eigen::Matrix4d& target) {
    Eigen::VectorXd error(6);

    // Position error
    error.head<3>() = target.block<3,1>(0,3) - current.block<3,1>(0,3);

    // Orientation error (axis-angle representation)
    Eigen::Matrix3d Re = target.block<3,3>(0,0) * current.block<3,3>(0,0).transpose();
    Eigen::AngleAxisd aa(Re);
    error.tail<3>() = aa.axis() * aa.angle();

    return error;
}

Eigen::MatrixXd PiperJacobianIK::computeNumericalJacobian(const std::vector<double>& joint_values) {
    const double delta = 1e-6;
    Eigen::MatrixXd J(6, 6);
    Eigen::Matrix4d T0 = fk_.computeFK(joint_values);
    Eigen::Vector3d p0 = T0.block<3,1>(0,3);

    for (int i = 0; i < 6; ++i) {
        // Perturb the ith joint
        std::vector<double> perturbed_values = joint_values;
        perturbed_values[i] += delta;

        // Compute FK for perturbed joint
        Eigen::Matrix4d Ti = fk_.computeFK(perturbed_values);
        Eigen::Vector3d pi = Ti.block<3,1>(0,3);

        // Position part of Jacobian (linear velocity)
        J.block<3,1>(0,i) = (pi - p0) / delta;

        // Orientation part of Jacobian (angular velocity)
        Eigen::Matrix3d dR = Ti.block<3,3>(0,0) * T0.block<3,3>(0,0).transpose();
        Eigen::AngleAxisd aa(dR);
        J.block<3,1>(3,i) = aa.axis() * aa.angle() / delta;
    }

    return J;
}

Eigen::MatrixXd PiperJacobianIK::computeAnalyticalJacobian(const std::vector<double>& joint_values, 
                                                          const Eigen::Matrix4d& current_pose) {
    Eigen::MatrixXd J(6, 6);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Vector3d> z_axes;
    std::vector<Eigen::Vector3d> p_ends;

    // Forward pass to compute all intermediate transforms and axes
    for (int i = 0; i < 6; ++i) {
        T = T * fk_.computeSingleTransform(i, joint_values[i]);
        z_axes.push_back(T.block<3,1>(0,2));  // z-axis of current frame
        p_ends.push_back(T.block<3,1>(0,3));  // origin of current frame
    }

    Eigen::Vector3d p_end = p_ends.back();  // end effector position

    // Compute Jacobian columns
    for (int i = 0; i < 6; ++i) {
        if (fk_.getDHParams()[i][2] == 0) {  // Revolute joint
            J.block<3,1>(0,i) = z_axes[i].cross(p_end - p_ends[i]);
            J.block<3,1>(3,i) = z_axes[i];
        } else {  // Prismatic joint (not used in Piper but included for completeness)
            J.block<3,1>(0,i) = z_axes[i];
            J.block<3,1>(3,i).setZero();
        }
    }

    return J;
}