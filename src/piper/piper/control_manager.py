#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from enum import Enum
import threading

class ControlMode(Enum):
    JOINT = 1
    CARTESIAN = 2

class ControlManager(Node):
    def __init__(self):
        super().__init__('control_manager')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Suscriptores
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/joint_commands_joi', self.joint_cmd_callback, 10)
        self.cartesian_cmd_sub = self.create_subscription(
            JointState, '/joint_commands_cart', self.cartesian_cmd_callback, 10)
        self.mode_sub = self.create_subscription(
            String, '/control_mode', self.mode_callback, 10)
        
        # Estado inicial - 8 joints en posición HOME
        self.current_mode = ControlMode.CARTESIAN
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # HOME position
        self.lock = threading.Lock()
        
        # Timer para publicación periódica (10Hz)
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        
        # Publicar estado inicial inmediatamente
        self.publish_joint_state()
        
        self.get_logger().info("Control Manager iniciado - Estado inicial publicado")

    def mode_callback(self, msg):
        with self.lock:
            if msg.data.lower() == "cartesian":
                self.current_mode = ControlMode.CARTESIAN
                self.get_logger().info("Modo cambiado a: CARTESIAN")
            elif msg.data.lower() == "joint":
                self.current_mode = ControlMode.JOINT
                self.get_logger().info("Modo cambiado a: JOINT")
        
    def joint_cmd_callback(self, msg):
        """Manejar comandos del modo JOINT"""
        if self.current_mode != ControlMode.JOINT:
            return
            
        with self.lock:
            self.get_logger().info(f"Recibido comando JOINT: {msg.position}")
            
            # El teleoperador envía 7 joints, adaptar a 8
            for i, name in enumerate(msg.name):
                idx = self.get_joint_index(name)
                if idx is not None:
                    if idx < 6:  # Joints 1-6 del brazo
                        self.current_joints[idx] = msg.position[i]
                    elif idx == 6:  # Gripper
                        self.current_joints[6] = msg.position[i]  # joint7
                        self.current_joints[7] = -msg.position[i]  # joint8
            
            self.get_logger().info(f"Joints actualizados: {self.current_joints}")

    def cartesian_cmd_callback(self, msg):
        """Manejar comandos del modo CARTESIANO"""
        if self.current_mode != ControlMode.CARTESIAN:
            return
            
        with self.lock:
            self.get_logger().info(f"Recibido comando CARTESIAN: {msg.position}")
            
            # El controlador cartesiano envía 8 joints
            for i, name in enumerate(msg.name):
                idx = self.get_joint_index(name)
                if idx is not None and idx < len(self.current_joints):
                    self.current_joints[idx] = msg.position[i]
            
            self.get_logger().info(f"Joints actualizados: {self.current_joints}")

    def publish_joint_state(self):
        """Publicar el estado actual de joints a /joint_states"""
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
            msg.position = self.current_joints.copy()
            
            self.joint_pub.publish(msg)
            self.get_logger().debug(f"Publicado joint state: {msg.position}", throttle_duration_sec=2.0)

    def get_joint_index(self, name):
        """Mapear nombre de joint a índice"""
        mapping = {
            'joint1': 0, 'joint2': 1, 'joint3': 2,
            'joint4': 3, 'joint5': 4, 'joint6': 5,
            'joint7': 6, 'joint8': 7
        }
        return mapping.get(name, None)

def main(args=None):
    rclpy.init(args=args)
    node = ControlManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()