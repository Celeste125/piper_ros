#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import select
import sys
import tty
import termios
import threading

class PiperKeyboardControl(Node):
    def __init__(self):
        super().__init__('piper_keyboard_control')
        
        # Publicadores para comandos
        self.delta_pub = self.create_publisher(Point, '/piper_cartesian_delta', 10)
        self.command_pub = self.create_publisher(String, '/piper_command', 10)
        
        # Iniciar control por teclado
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('🎮 Keyboard Controller INICIADO')
        self.print_controls()
        
    def print_controls(self):
        """Imprime controles del teclado"""
        controls = """
        CONTROLES DE TECLADO:
        -------------------------
        Movimiento Cartesianos (1mm):
          w: +Y (adelante)    s: -Y (atrás)
          a: -X (izquierda)   d: +X (derecha)
          r: +Z (arriba)      f: -Z (abajo)
        
        Movimiento Cartesianos (5mm):
          t: +Y (5mm)         g: -Y (5mm)
          h: -X (5mm)         j: +X (5mm)
          u: +Z (5mm)         m: -Z (5mm)
        
        Comandos:
          o: Posición inicial
          Q: Salir
        """
        print(controls)
    
    def send_delta_command(self, dx, dy, dz):
        """Envía comando de desplazamiento"""
        msg = Point()
        msg.x = dx
        msg.y = dy 
        msg.z = dz
        self.delta_pub.publish(msg)
        self.get_logger().info(f'Delta enviado: Δx={dx:.3f}, Δy={dy:.3f}, Δz={dz:.3f}')
    
    def send_home_command(self):
        """Envía comando HOME"""
        msg = String()
        msg.data = 'home'
        self.command_pub.publish(msg)
        self.get_logger().info('🏠 Comando HOME enviado')
    
    def keyboard_loop(self):
        """Loop principal de control por teclado"""
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    # Movimiento de 1mm
                    if key == 'w':
                        self.send_delta_command(0.0, 0.001, 0.0)
                    elif key == 's':
                        self.send_delta_command(0.0, -0.001, 0.0)
                    elif key == 'a':
                        self.send_delta_command(-0.001, 0.0, 0.0)
                    elif key == 'd':
                        self.send_delta_command(0.001, 0.0, 0.0)
                    elif key == 'r':
                        self.send_delta_command(0.0, 0.0, 0.001)
                    elif key == 'f':
                        self.send_delta_command(0.0, 0.0, -0.001)
                    
                    # Movimiento de 5mm
                    elif key == 't':
                        self.send_delta_command(0.0, 0.01, 0.0)
                    elif key == 'g':
                        self.send_delta_command(0.0, -0.01, 0.0)
                    elif key == 'h':
                        self.send_delta_command(-0.01, 0.0, 0.0)
                    elif key == 'j':
                        self.send_delta_command(0.01, 0.0, 0.0)
                    elif key == 'u':
                        self.send_delta_command(0.0, 0.0, 0.01)
                    elif key == 'm':
                        self.send_delta_command(0.0, 0.0, -0.01)
                    
                    # Comandos especiales
                    elif key == 'o':
                        self.send_home_command()
                    elif key == 'q':
                        self.get_logger().info('👋 Saliendo...')
                        break
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = PiperKeyboardControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()