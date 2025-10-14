#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class DeltaManager(Node):
    def __init__(self):
        super().__init__('delta_manager')
        
        # Suscriptor de deltas
        self.delta_sub = self.create_subscription(
            Float64MultiArray, '/joint_commands_delta', self.delta_callback, 10)
        
        # Publicador de comandos absolutos
        self.joint_pub = self.create_publisher(JointState, '/joint_commands_joi', 10)
        
        # Estado actual (se actualiza con joint_states)
        self.current_joints = [0.0] * 7  # 6 joints brazo + 1 gripper
        self.joint_state_received = False
        
        # Suscriptor a joint_states para conocer posición actual
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Límites articulares
        self.joint_limits = [
            (-2.618, 2.618),    # Joint 1
            (0.0, math.pi),     # Joint 2  
            (-math.pi, 0.0),    # Joint 3
            (-2.967, 2.967),    # Joint 4
            (-1.2, 1.2),        # Joint 5
            (-1.22, 1.22),      # Joint 6
            (0.0, 0.04)         # Gripper
        ]
        
        self.get_logger().info("Delta Manager iniciado")
        self.get_logger().info("Escuchando deltas en: /joint_commands_delta")
        self.get_logger().info("Publicando comandos absolutos en: /joint_commands_joi")

    def joint_state_callback(self, msg):
        """Actualizar posición actual del robot"""
        if not self.joint_state_received:
            self.joint_state_received = True
            self.get_logger().info("Posición actual recibida")
        
        # Extraer primeros 7 joints (joint8 es complementario de joint7)
        for i, name in enumerate(msg.name):
            if i < 7:  # Solo nos interesan los primeros 7
                idx = self.get_joint_index(name)
                if idx is not None and idx < len(self.current_joints):
                    self.current_joints[idx] = msg.position[i]

    def delta_callback(self, msg):
        """Convertir deltas a comandos absolutos"""
        if not self.joint_state_received:
            self.get_logger().warn("Delta ignorado - posición actual desconocida")
            return
            
        if len(msg.data) != 7:
            self.get_logger().error(f"Delta inválido. Esperaba 7 valores, recibió {len(msg.data)}")
            return
        
        self.get_logger().info(f"🔄 Procesando delta: {[f'{d:.3f}' for d in msg.data]}")
        
        # Calcular nueva posición absoluta
        new_joints = self.current_joints.copy()
        
        # Aplicar deltas a joints 1-6
        for i in range(6):
            new_joints[i] += msg.data[i]
            # Aplicar límites
            new_joints[i] = max(self.joint_limits[i][0], 
                              min(self.joint_limits[i][1], new_joints[i]))
        
        # Aplicar delta al gripper
        new_joints[6] += msg.data[6]
        new_joints[6] = max(0.0, min(0.04, new_joints[6]))
        
        # Publicar comando absoluto
        self.publish_absolute_command(new_joints)
        
        # Actualizar posición actual
        self.current_joints = new_joints
        
        # Log de movimiento
        joint_moves = [f"J{i+1}:{msg.data[i]:+.3f}" for i in range(6) if abs(msg.data[i]) > 0.001]
        gripper_move = f"Gripper:{msg.data[6]:+.3f}" if abs(msg.data[6]) > 0.001 else ""
        moves = [m for m in joint_moves + [gripper_move] if m]
        if moves:
            self.get_logger().info(f"➡️  Movimiento: {', '.join(moves)}")

    def publish_absolute_command(self, joints):
        """Publicar comando absoluto"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        msg.position = joints
        
        self.joint_pub.publish(msg)

    def get_joint_index(self, name):
        """Mapear nombre de joint a índice"""
        mapping = {
            'joint1': 0, 'joint2': 1, 'joint3': 2,
            'joint4': 3, 'joint5': 4, 'joint6': 5,
            'joint7': 6
        }
        return mapping.get(name, None)

def main(args=None):
    rclpy.init(args=args)
    node = DeltaManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()