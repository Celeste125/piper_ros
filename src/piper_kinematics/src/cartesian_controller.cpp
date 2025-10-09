#include "piper_kinematics/piper_jacobian_ik.hpp"
#include "piper_kinematics/piper_forward_kinematics.hpp"
#include "piper_kinematics/piper_analytical_ik.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <deque>
#include <algorithm>

class CartesianController : public rclcpp::Node {
public:
    CartesianController() : Node("cartesian_controller") {
        // Declarar parámetros
        this->declare_parameter<std::string>("dh_type", "modified");
        this->declare_parameter<int>("max_iterations", 50);
        this->declare_parameter<double>("position_tolerance", 1e-3);
        this->declare_parameter<double>("orientation_tolerance", 1e-2);
        this->declare_parameter<double>("damping_factor", 0.1);
        this->declare_parameter<double>("publish_rate", 30.0);
        this->declare_parameter<bool>("use_analytic_ik", false);
        
        // Inicializar solvers IK
        std::string dh_type_str = this->get_parameter("dh_type").as_string();
        PiperForwardKinematics::DHType dh_type = (dh_type_str == "standard") ? 
            PiperForwardKinematics::STANDARD : PiperForwardKinematics::MODIFIED;
        
        ik_numerical_solver_ = std::make_unique<PiperJacobianIK>(dh_type);
        ik_analytical_solver_ = std::make_unique<PiperAnalyticalIK>(dh_type);
        
        // Configurar solvers
        ik_numerical_solver_->setMaxIterations(this->get_parameter("max_iterations").as_int());
        ik_numerical_solver_->setPositionTolerance(this->get_parameter("position_tolerance").as_double());
        ik_numerical_solver_->setOrientationTolerance(this->get_parameter("orientation_tolerance").as_double());
        ik_numerical_solver_->setDampingFactor(this->get_parameter("damping_factor").as_double());
        
        // Establecer límites articulares (SOLO para brazo - joints 1-6)
        joint_limits_ = {
            {-2.618, 2.618},    // Joint 1
            {0, M_PI},          // Joint 2
            {-M_PI, 0},         // Joint 3
            {-2.967, 2.967},    // Joint 4
            {-1.2, 1.2},       // Joint 5
            {-1.22, 1.22}      // Joint 6
        };
        ik_numerical_solver_->setJointLimits(joint_limits_);
        ik_analytical_solver_->setJointLimits(joint_limits_);
        
        // Estado inicial
        // 6 joints para brazo + 2 joints para gripper
        arm_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_position_ = 0.0; // 0 = cerrado, 0.035 = abierto
        previous_solution_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
        
        // Publicadores
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        ik_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/ik_status", 10);
        
        // Suscriptores
        cartesian_target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/piper_cartesian_target", 10,
            std::bind(&CartesianController::cartesianTargetCallback, this, std::placeholders::_1));
            
        cartesian_delta_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/piper_cartesian_delta", 10,
            std::bind(&CartesianController::cartesianDeltaCallback, this, std::placeholders::_1));
            
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/piper_command", 10,
            std::bind(&CartesianController::commandCallback, this, std::placeholders::_1));
            
        gripper_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gripper_command", 10,
            std::bind(&CartesianController::gripperCallback, this, std::placeholders::_1));
            
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CartesianController::jointStateCallback, this, std::placeholders::_1));
        
        // Timer para publicación continua
        double publish_rate = this->get_parameter("publish_rate").as_double();
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&CartesianController::publishJointState, this));
        
        // Broadcaster TF
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "Cartesian Controller initialized");
    }

private:
    std::unique_ptr<PiperJacobianIK> ik_numerical_solver_;
    std::unique_ptr<PiperAnalyticalIK> ik_analytical_solver_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ik_status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cartesian_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cartesian_delta_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::vector<double> arm_joint_positions_;  // Solo joints 1-6 (brazo)
    std::vector<double> previous_solution_;    // Solo joints 1-6
    double gripper_position_;                  // Posición del gripper (0-0.035)
    std::vector<std::string> joint_names_;
    std::vector<std::pair<double, double>> joint_limits_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        try {
            // Actualizar posiciones del brazo (joints 1-6)
            for (size_t i = 0; i < 6; ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                if (it != msg->name.end()) {
                    size_t idx = std::distance(msg->name.begin(), it);
                    arm_joint_positions_[i] = msg->position[idx];
                }
            }
            
            // Actualizar posición del gripper (usar joint7 como referencia)
            auto gripper_it = std::find(msg->name.begin(), msg->name.end(), "joint7");
            if (gripper_it != msg->name.end()) {
                size_t idx = std::distance(msg->name.begin(), gripper_it);
                gripper_position_ = msg->position[idx];
            }
            
            // Actualizar solución anterior (solo brazo)
            previous_solution_ = ensureAngleContinuity(previous_solution_, arm_joint_positions_);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in joint state callback: %s", e.what());
        }
    }

    void gripperCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Controlar el gripper (0 = cerrado, 0.035 = abierto)
        gripper_position_ = std::clamp(msg->data, 0.0, 0.035);
        RCLCPP_INFO(this->get_logger(), "Gripper command: %.3f", gripper_position_);
    }

    void cartesianTargetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        try {
            RCLCPP_INFO(this->get_logger(), "Cartesian target: x=%.3f, y=%.3f, z=%.3f", 
                       msg->x, msg->y, msg->z);
            
            // Crear pose objetivo (mantener orientación actual)
            Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
            target_pose(0, 3) = msg->x;
            target_pose(1, 3) = msg->y;
            target_pose(2, 3) = msg->z;
            
            // Usar orientación actual
            PiperForwardKinematics fk(PiperForwardKinematics::MODIFIED);
            Eigen::Matrix4d current_pose = fk.computeFK(arm_joint_positions_);
            target_pose.block<3, 3>(0, 0) = current_pose.block<3, 3>(0, 0);
            
            // Calcular IK (solo para brazo - joints 1-6)
            std::vector<double> solution;
            if (this->get_parameter("use_analytic_ik").as_bool()) {
                solution = ik_analytical_solver_->findBestSolution(target_pose, arm_joint_positions_);
            } else {
                solution = ik_numerical_solver_->computeIK(previous_solution_, target_pose);
            }
            
            // Aplicar continuidad angular
            solution = ensureAngleContinuity(previous_solution_, solution);
            
            // Actualizar estado del brazo
            arm_joint_positions_ = solution;
            previous_solution_ = solution;
            
            // Publicar estado IK
            auto ik_status = std_msgs::msg::Bool();
            ik_status.data = true;
            ik_status_pub_->publish(ik_status);
            
            RCLCPP_INFO(this->get_logger(), "IK solution found successfully");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IK failed: %s", e.what());
            
            auto ik_status = std_msgs::msg::Bool();
            ik_status.data = false;
            ik_status_pub_->publish(ik_status);
        }
    }

    void cartesianDeltaCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        try {
            RCLCPP_INFO(this->get_logger(), "Cartesian delta: dx=%.3f, dy=%.3f, dz=%.3f", 
                       msg->x, msg->y, msg->z);
            
            // Obtener posición actual
            PiperForwardKinematics fk(PiperForwardKinematics::MODIFIED);
            Eigen::Matrix4d current_pose = fk.computeFK(arm_joint_positions_);
            
            // Calcular nueva posición
            Eigen::Vector3d new_position = current_pose.block<3, 1>(0, 3) + 
                                         Eigen::Vector3d(msg->x, msg->y, msg->z);
            
            // Crear nueva pose objetivo
            Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
            target_pose.block<3, 1>(0, 3) = new_position;
            target_pose.block<3, 3>(0, 0) = current_pose.block<3, 3>(0, 0);
            
            // Calcular IK (solo para brazo)
            std::vector<double> solution;
            if (this->get_parameter("use_analytic_ik").as_bool()) {
                solution = ik_analytical_solver_->findBestSolution(target_pose, arm_joint_positions_);
            } else {
                solution = ik_numerical_solver_->computeIK(previous_solution_, target_pose);
            }
            
            // Aplicar continuidad angular
            solution = ensureAngleContinuity(previous_solution_, solution);
            
            // Actualizar estado del brazo
            arm_joint_positions_ = solution;
            previous_solution_ = solution;
            
            RCLCPP_INFO(this->get_logger(), "Delta movement completed");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Delta movement failed: %s", e.what());
        }
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);
        
        if (command == "home") {
            RCLCPP_INFO(this->get_logger(), "Home command received");
            arm_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            previous_solution_ = arm_joint_positions_;
            gripper_position_ = 0.0; // Gripper cerrado
        } else if (command == "open_gripper") {
            gripper_position_ = 0.035; // Gripper abierto
        } else if (command == "close_gripper") {
            gripper_position_ = 0.0; // Gripper cerrado
        }
    }

    void publishJointState() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        // Combinar brazo (6 joints) + gripper (2 joints)
        std::vector<double> all_positions = arm_joint_positions_;
        
        // Gripper: joint7 y joint8 son prismáticos complementarios
        // joint7: 0 a 0.035 (positivo)
        // joint8: -0.035 a 0 (negativo)
        all_positions.push_back(gripper_position_);                    // joint7
        all_positions.push_back(-gripper_position_);                   // joint8
        
        msg.position = all_positions;
        joint_state_pub_->publish(msg);
    }

    std::vector<double> ensureAngleContinuity(const std::vector<double>& prev_angles, 
                                             const std::vector<double>& new_angles) {
        if (prev_angles.size() != new_angles.size()) {
            return new_angles;
        }
        
        std::vector<double> continuous_angles = new_angles;
        
        for (size_t i = 0; i < prev_angles.size(); ++i) {
            double diff = new_angles[i] - prev_angles[i];
            
            // Si el cambio de ángulo excede π, ajustar por ±2π para minimizar cambio
            if (diff > M_PI) {
                continuous_angles[i] -= 2 * M_PI;
            } else if (diff < -M_PI) {
                continuous_angles[i] += 2 * M_PI;
            }
            
            // Asegurar que sigue dentro de los límites articulares
            continuous_angles[i] = std::clamp(continuous_angles[i], 
                                            joint_limits_[i].first, 
                                            joint_limits_[i].second);
        }
        
        return continuous_angles;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}