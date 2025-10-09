#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import math
import time

class PiperCartesianController(Node):
    def __init__(self):
        super().__init__('piper_cartesian_controller')
        
        # Parámetros DH CORREGIDOS para AgileX Piper (en metros)
        self.dh_params = [
            # Joint 1: Base rotativa
            {'theta': 0,      'd': 0.123,  'a': 0.000,  'alpha': -math.pi/2},  # d1 = 130mm
            # Joint 2: Hombro
            {'theta': -177.22/180*math.pi,      'd': 0.000,  'a': 0.28505,  'alpha': 0},          # a2 = 180mm  
            # Joint 3: Codo
            {'theta': -102.78/180*math.pi,      'd': 0.000,  'a': -0.02198,  'alpha': math.pi/2}, # a3 = 60mm
            # Joint 4: Muñeca 1
            {'theta': 0,      'd': 0.25065,  'a': 0,  'alpha': -math.pi/2},  # d4 = 180mm
            # Joint 5: Muñeca 2  
            {'theta': 0,      'd': 0.000,  'a': 0.000,  'alpha': math.pi/2},
            # Joint 6: Muñeca 3
            {'theta': 0,      'd': 0.091,  'a': 0.000,  'alpha': 0}           # d6 = 65mm
        ]
        
        # Estado actual de las articulaciones
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
        
        # Posición HOME del robot
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publicador para COMANDOS de joint
        self.joint_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Suscriptor para coordenadas Cartesianas
        self.cartesian_subscriber = self.create_subscription(
            Point,
            '/piper_cartesian_target',
            self.cartesian_target_callback,
            10
        )
        
        # Suscriptor para desplazamientos cartesianos
        self.cartesian_delta_subscriber = self.create_subscription(
            Point,
            '/piper_cartesian_delta',
            self.cartesian_delta_callback,
            10
        )
        
        # Suscriptor para estado actual de joints
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publicar posición HOME al iniciar
        self.publish_home_position()
        
        self.get_logger().info('🚀 Piper Cartesian Controller INICIADO')
        self.get_logger().info('📡 Escuchando en: /piper_cartesian_target y /piper_cartesian_delta')
        
    def publish_home_position(self):
        """Publica posición HOME al iniciar"""
        time.sleep(1.0)
        self.send_joint_command(self.home_position)
        self.get_logger().info('🏠 Posición HOME enviada')
    
    def joint_state_callback(self, msg):
        """Actualiza el estado actual de las articulaciones"""
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_joint_positions[i] = msg.position[idx]
            
        except Exception as e:
            self.get_logger().error(f'Error updating joint states: {e}')
    
    def cartesian_target_callback(self, msg):
        """Callback para recibir coordenadas Cartesianas absolutas"""
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'🎯 Recibido target ABSOLUTO: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Calcular cinemática inversa directa
        target_joints = self.calculate_inverse_kinematics(x, y, z)
        
        if target_joints is not None:
            self.send_joint_command(target_joints)
        else:
            self.get_logger().error('❌ No se pudo calcular la cinemática inversa')
    
    def cartesian_delta_callback(self, msg):
        """Callback para recibir desplazamientos cartesianos relativos"""
        dx, dy, dz = msg.x, msg.y, msg.z
        self.get_logger().info(f'🎯 Recibido DESPLAZAMIENTO: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}')
        
        # Mover suavemente con desplazamiento cartesiano
        self.mover_cartesiano(dx, dy, dz, pasos=30)
    
    def matriz_transformacion_DH(self, theta, d, a, alpha):
        """Matriz de transformación Denavit-Hartenberg"""
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def cinematica_directa(self, q):
        """Cinemática directa - calcula posición del efector final"""
        T = np.eye(4)
        
        for i in range(6):
            theta = self.dh_params[i]['theta'] + q[i]
            d = self.dh_params[i]['d']
            a = self.dh_params[i]['a'] 
            alpha = self.dh_params[i]['alpha']
            
            T_i = self.matriz_transformacion_DH(theta, d, a, alpha)
            T = T @ T_i
        
        return T[:3, 3]  # Retorna posición [x, y, z]
    
    def jacobiano_geometrico(self, q, delta=0.0001):
        """Calcula el jacobiano geométrico (solo posición)"""
        jacobiano = np.zeros((3, 6))
        pos_actual = self.cinematica_directa(q)
        
        for i in range(6):
            q_pert = q.copy()
            q_pert[i] += delta
            pos_pert = self.cinematica_directa(q_pert)
            jacobiano[:, i] = (pos_pert - pos_actual) / delta
        
        return jacobiano
    
    def calculate_inverse_kinematics(self, x, y, z, max_iter=200, tol=1e-4):
        """Cinemática inversa usando método numérico iterativo"""
        q_current = np.array(self.current_joint_positions[0:6])
        target_pos = np.array([x, y, z])
        
        self.get_logger().info(f'🔍 IK - Objetivo: [{x:.3f}, {y:.3f}, {z:.3f}]')
        
        # Algoritmo iterativo
        for iteracion in range(max_iter):
            current_pos = self.cinematica_directa(q_current)
            error = target_pos - current_pos
            error_norm = np.linalg.norm(error)
            
            if error_norm < tol:
                self.get_logger().info(f'✅ IK - Convergió en {iteracion} iteraciones')
                result = q_current.tolist() + self.current_joint_positions[6:8]
                return result
            
            J = self.jacobiano_geometrico(q_current)
            
            try:
                J_pinv = np.linalg.pinv(J)
                dq = J_pinv @ error
                q_current = q_current + dq * 0.3  # Factor de ganancia reducido
                q_current = np.arctan2(np.sin(q_current), np.cos(q_current))
                
            except Exception as e:
                self.get_logger().error(f'❌ Error en cálculo IK: {e}')
                return None
        
        self.get_logger().warning(f'⚠️ IK - No convergió después de {max_iter} iteraciones')
        return None

    def mover_cartesiano(self, dx, dy, dz, pasos=30):
        """Mueve el efector final un pequeño desplazamiento cartesiano de forma suave."""
        self.get_logger().info(f'🔄 Movimiento suave: Δx={dx:.3f}, Δy={dy:.3f}, Δz={dz:.3f}, pasos={pasos}')
        
        q_actual = np.array(self.current_joint_positions[0:6])
        pos_actual = self.cinematica_directa(q_actual)

        # Trayectoria lineal en el espacio cartesiano
        for i in range(1, pasos + 1):
            # Nueva posición objetivo intermedia
            factor = i / pasos
            pos_intermedia = pos_actual + np.array([dx, dy, dz]) * factor
            
            # IK numérica usando la configuración anterior como semilla
            q_obj = self.calculate_inverse_kinematics(
                pos_intermedia[0], 
                pos_intermedia[1], 
                pos_intermedia[2]
            )
            
            if q_obj is None:
                self.get_logger().warning('⚠️ No convergió en paso intermedio, deteniendo movimiento')
                break
            
            # Publicar posición intermedia
            self.send_joint_command(q_obj)
            time.sleep(0.03)  # Suaviza el movimiento (30ms entre pasos)
            
            # Actualizar posición actual para el siguiente paso
            q_actual = np.array(q_obj[0:6])
        
        self.get_logger().info('✅ Movimiento suave completado')
    
    def send_joint_command(self, joint_positions):
        """Envía comandos de posición a las articulaciones"""
        try:
            # Límites de seguridad para joints 1-6
            joint_limits = [
                [-3.14, 3.14], [-2.35, 2.35], [-3.14, 3.14],
                [-3.14, 3.14], [-3.14, 3.14], [-3.14, 3.14]
            ]
            
            # Verificar límites para joints 1-6
            for i in range(6):
                pos = joint_positions[i]
                if pos < joint_limits[i][0] or pos > joint_limits[i][1]:
                    self.get_logger().error(f'❌ Joint {i+1} fuera de límites: {pos:.3f} rad')
                    return
            
            # Crear mensaje JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [float(pos) for pos in joint_positions]
            
            # Publicar comando
            self.joint_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando comando: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = PiperCartesianController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Controller apagado por usuario')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()