#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import termios
import tty
import sys
import select
import math

class PiperKeyboardControl(Node):
    def __init__(self):
        super().__init__('piper_keyboard_control')
        
        # Publicador para la pose goal
        self.pose_publisher = self.create_publisher(Pose, '/end_pose', 10)
        
        # Pose inicial (basada en la que viste al hacer echo)
        self.current_pose = Pose()
        self.current_pose.position.x = 0.05
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.25
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.675
        self.current_pose.orientation.z = 0.0
        self.current_pose.orientation.w = 0.737
        
        # Parámetros de movimiento
        self.position_step = 0.01  # 1 cm
        self.orientation_step = 0.1  # radianes
        
        self.print_controls()
        
    def print_controls(self):
        print("\n=== CONTROL POR TECLADO - PIPER ROBOT ===")
        print("Movimiento de POSICIÓN:")
        print("  w/s: mover adelante/atrás (X)")
        print("  a/d: mover izquierda/derecha (Y)")
        print("  q/e: mover arriba/abajo (Z)")
        print("")
        print("Movimiento de ORIENTACIÓN:")
        print("  i/k: rotar alrededor de X (pitch)")
        print("  j/l: rotar alrededor de Y (roll)") 
        print("  u/o: rotar alrededor de Z (yaw)")
        print("")
        print("Otras funciones:")
        print("  r: reset a posición inicial")
        print("  p: publicar pose actual")
        print("  ESC: salir")
        print("==========================================\n")
        self.print_current_pose()
    
    def print_current_pose(self):
        print(f"Posición: X={self.current_pose.position.x:.3f}, "
              f"Y={self.current_pose.position.y:.3f}, "
              f"Z={self.current_pose.position.z:.3f}")
        print(f"Orientación: w={self.current_pose.orientation.w:.3f}, "
              f"x={self.current_pose.orientation.x:.3f}, "
              f"y={self.current_pose.orientation.y:.3f}, "
              f"z={self.current_pose.orientation.z:.3f}")
        print()
    
    def get_key(self):
        """Lee una tecla sin bloquear"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            [i, o, e] = select.select([sys.stdin], [], [], 0.1)
            if i:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def normalize_quaternion(self, q):
        """Normaliza un cuaternión"""
        length = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if length > 0:
            q.w /= length
            q.x /= length
            q.y /= length
            q.z /= length
        return q
    
    def rotate_quaternion(self, q, axis, angle):
        """Rota un cuaternión alrededor de un eje"""
        # Crear cuaternión de rotación
        half_angle = angle / 2.0
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)
        
        if axis == 'x':
            rot_q = Quaternion(x=sin_half, y=0.0, z=0.0, w=cos_half)
        elif axis == 'y':
            rot_q = Quaternion(x=0.0, y=sin_half, z=0.0, w=cos_half)
        elif axis == 'z':
            rot_q = Quaternion(x=0.0, y=0.0, z=sin_half, w=cos_half)
        else:
            return q
        
        # Multiplicar cuaterniones: q_result = q * rot_q
        result = Quaternion()
        result.w = q.w * rot_q.w - q.x * rot_q.x - q.y * rot_q.y - q.z * rot_q.z
        result.x = q.w * rot_q.x + q.x * rot_q.w + q.y * rot_q.z - q.z * rot_q.y
        result.y = q.w * rot_q.y - q.x * rot_q.z + q.y * rot_q.w + q.z * rot_q.x
        result.z = q.w * rot_q.z + q.x * rot_q.y - q.y * rot_q.x + q.z * rot_q.w
        
        return self.normalize_quaternion(result)
    
    def run(self):
        print("Iniciando control por teclado...")
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    print("\nSaliendo...")
                    break
                
                # Movimiento de posición
                elif key == 'w':  # +X
                    self.current_pose.position.x += self.position_step
                elif key == 's':  # -X
                    self.current_pose.position.x -= self.position_step
                elif key == 'a':  # +Y
                    self.current_pose.position.y += self.position_step
                elif key == 'd':  # -Y
                    self.current_pose.position.y -= self.position_step
                elif key == 'q':  # +Z
                    self.current_pose.position.z += self.position_step
                elif key == 'e':  # -Z
                    self.current_pose.position.z -= self.position_step
                
                # Movimiento de orientación
                elif key == 'i':  # +X rotation (pitch down)
                    self.current_pose.orientation = self.rotate_quaternion(
                        self.current_pose.orientation, 'x', self.orientation_step)
                elif key == 'k':  # -X rotation (pitch up)
                    self.current_pose.orientation = self.rotate_quaternion(
                        self.current_pose.orientation, 'x', -self.orientation_step)
                elif key == 'j':  # +Y rotation (roll left)
                    self.current_pose.orientation = self.rotate_quaternion(
                        self.current_pose.orientation, 'y', self.orientation_step)
                elif key == 'l':  # -Y rotation (roll right)
                    self.current_pose.orientation = self.rotate_quaternion(
                        self.current_pose.orientation, 'y', -self.orientation_step)
                elif key == 'u':  # +Z rotation (yaw left)
                    self.current_pose.orientation = self.rotate_quaternion(
                        self.current_pose.orientation, 'z', self.orientation_step)
                elif key == 'o':  # -Z rotation (yaw right)
                    self.current_pose.orientation = self.rotate_quaternion(
                        self.current_pose.orientation, 'z', -self.orientation_step)
                
                # Funciones especiales
                elif key == 'r':  # Reset
                    self.current_pose.position.x = 0.05
                    self.current_pose.position.y = 0.0
                    self.current_pose.position.z = 0.25
                    self.current_pose.orientation.x = 0.0
                    self.current_pose.orientation.y = 0.675
                    self.current_pose.orientation.z = 0.0
                    self.current_pose.orientation.w = 0.737
                    print("Posición reseteada")
                
                elif key == 'p':  # Publicar sin mover
                    print("Publicando pose actual...")
                
                # Publicar si hubo cambio o se presionó 'p'
                if key in ['w','s','a','d','q','e','i','k','j','l','u','o','r','p']:
                    self.pose_publisher.publish(self.current_pose)
                    self.print_current_pose()
                        
        except Exception as e:
            print(f"Error: {e}")
        finally:
            print("Control por teclado terminado")

def main():
    rclpy.init()
    node = PiperKeyboardControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()