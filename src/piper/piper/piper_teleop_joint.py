#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, termios, tty, select, math
from std_msgs.msg import String


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class PiperTeleopJoint(Node):
    def __init__(self):
        super().__init__('piper_teleop_joint')
        self.pub = self.create_publisher(JointState, '/joint_commands_joi', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        
        # Suscriptor para leer la posición actual del robot
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.current_mode = "cartesian"
        
        self.timer = self.create_timer(0.03, self.update_cmd)

        # Inicializar con zeros - se actualizará con la posición real
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state_received = False
        
        # Límites de las articulaciones (radianes)
        self.joint_limits = [
            (-2.618, 2.618),    # Joint 1
            (0.0, math.pi),     # Joint 2  
            (-math.pi, 0.0),    # Joint 3
            (-2.967, 2.967),    # Joint 4
            (-1.2, 1.2),        # Joint 5
            (-1.22, 1.22),      # Joint 6
            (0.0, 0.04)         # Gripper
        ]
        
        self.step = 0.05  # incremento por tecla (rad)
        self.gripper_effort = 1.0
        
        self.get_logger().info("Esperando posición actual del robot...")

    def switch_mode(self):
        self.current_mode = "cartesian" if self.current_mode == "joint" else "joint"
        mode_msg = String(data=self.current_mode)
        self.mode_pub.publish(mode_msg)
        self.get_logger().info(f"Modo: {self.current_mode.upper()}")

    def joint_state_callback(self, msg):
        """Callback que se ejecuta cuando llega la posición actual del robot"""
        if not self.joint_state_received:
            self.joints = list(msg.position)
            self.joint_state_received = True
            self.get_logger().info(f"Posición actual recibida: {[f'{x:.3f}' for x in self.joints]}")
            self.get_logger().info("Teleop Piper iniciado. Usa Q/A, W/S, E/D, R/F, T/G, Y/H, U/J para joints 1–7, ESC para salir.")

    def apply_joint_limits(self, joint_idx, value):
        """Aplica límites a la articulación"""
        min_val, max_val = self.joint_limits[joint_idx]
        return max(min_val, min(value, max_val))

    def update_cmd(self):
        # Si aún no hemos recibido la posición actual, no hacer nada
        if not self.joint_state_received:
            return

        settings = termios.tcgetattr(sys.stdin)
        key = get_key(settings)

        if key == '\x1b':  # ESC
            self.get_logger().info("Cerrando teleoperador.")
            raise KeyboardInterrupt

        # joint 1
        if key == 'q': 
            self.joints[0] = self.apply_joint_limits(0, self.joints[0] + self.step)
        if key == 'a': 
            self.joints[0] = self.apply_joint_limits(0, self.joints[0] - self.step)
            
        # joint 2
        if key == 'w': 
            self.joints[1] = self.apply_joint_limits(1, self.joints[1] + self.step)
        if key == 's': 
            self.joints[1] = self.apply_joint_limits(1, self.joints[1] - self.step)
            
        # joint 3
        if key == 'e': 
            self.joints[2] = self.apply_joint_limits(2, self.joints[2] + self.step)
        if key == 'd': 
            self.joints[2] = self.apply_joint_limits(2, self.joints[2] - self.step)
            
        # joint 4
        if key == 'r': 
            self.joints[3] = self.apply_joint_limits(3, self.joints[3] + self.step)
        if key == 'f': 
            self.joints[3] = self.apply_joint_limits(3, self.joints[3] - self.step)
            
        # joint 5
        if key == 't': 
            self.joints[4] = self.apply_joint_limits(4, self.joints[4] + self.step)
        if key == 'g': 
            self.joints[4] = self.apply_joint_limits(4, self.joints[4] - self.step)
            
        # joint 6
        if key == 'y': 
            self.joints[5] = self.apply_joint_limits(5, self.joints[5] + self.step)
        if key == 'h': 
            self.joints[5] = self.apply_joint_limits(5, self.joints[5] - self.step)
            
        # gripper
        if key == 'u': 
            self.joints[6] = min(self.joints[6] + 0.005, 0.04)
        if key == 'j': 
            self.joints[6] = max(self.joints[6] - 0.005, 0.0)

        if key == 'm': 
            self.switch_mode()

        # PUBLICAR SIEMPRE (cada 30ms) - mantiene la posición actual
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        msg.position = self.joints
        msg.velocity = [0.0]*6 + [10.0]
        msg.effort = [0.0]*6 + [self.gripper_effort]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PiperTeleopJoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()