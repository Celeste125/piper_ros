#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        
        self.get_logger().info(" GameSir Controller - Modo Simple (Solo Botones)")
        
        # Suscriptor para el gamepad
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publicador para comandos cartesianos
        self.cartesian_pub = self.create_publisher(
            Point, 
            '/piper_cartesian_delta', 
            10
        )
        
        # Configuración de botones según tu especificación
        self.button_map = {
            'z_up': 4,      # Botón 4: Subir Z +0.01
            'z_down': 0,    # Botón 0: Bajar Z -0.01
            'x_forward': 1, # Botón 1: Avanzar X +0.01  
            'x_backward': 3 # Botón 3: Retroceder X -0.01
        }
        
        # Delta fijo de movimiento
        self.delta = 0.01
        
        # Estado para detectar pulsaciones
        self.last_buttons = [0] * 16
        
        self.get_logger().info("✅ Controlador simple listo")
        self.get_logger().info("📋 Controles:")
        self.get_logger().info("  - Botón 4: Z +0.01 (Subir)")
        self.get_logger().info("  - Botón 0: Z -0.01 (Bajar)")
        self.get_logger().info("  - Botón 1: X +0.01 (Avanzar)")
        self.get_logger().info("  - Botón 3: X -0.01 (Retroceder)")

    def joy_callback(self, msg):
        """Procesa los botones y publica deltas cartesianos"""
        
        # Crear mensaje de delta cartesiano
        cartesian_delta = Point()
        cartesian_delta.x = 0.0
        cartesian_delta.y = 0.0
        cartesian_delta.z = 0.0
        
        movement_detected = False
        
        # Verificar cada botón (solo cuando se presiona, no cuando se mantiene)
        
        # Botón 4: Subir Z
        if msg.buttons[self.button_map['z_up']] == 1 and self.last_buttons[self.button_map['z_up']] == 0:
            cartesian_delta.z = self.delta
            movement_detected = True
            self.get_logger().info(f"⬆️  Z +{self.delta}")
        
        # Botón 0: Bajar Z  
        elif msg.buttons[self.button_map['z_down']] == 1 and self.last_buttons[self.button_map['z_down']] == 0:
            cartesian_delta.z = -self.delta
            movement_detected = True
            self.get_logger().info(f"⬇️  Z -{self.delta}")
        
        # Botón 1: Avanzar X
        elif msg.buttons[self.button_map['x_forward']] == 1 and self.last_buttons[self.button_map['x_forward']] == 0:
            cartesian_delta.x = self.delta
            movement_detected = True
            self.get_logger().info(f"➡️  X +{self.delta}")
        
        # Botón 3: Retroceder X
        elif msg.buttons[self.button_map['x_backward']] == 1 and self.last_buttons[self.button_map['x_backward']] == 0:
            cartesian_delta.x = -self.delta
            movement_detected = True
            self.get_logger().info(f"⬅️  X -{self.delta}")
        
        # Publicar solo si hay movimiento
        if movement_detected:
            self.cartesian_pub.publish(cartesian_delta)
        
        # Guardar estado actual para la siguiente llamada
        self.last_buttons = list(msg.buttons)  # Convertir a lista primero

def main():
    rclpy.init()
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Apagando controlador simple...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()