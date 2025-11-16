#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
import pygame
import os

class PiperTeleopGamepad(Node):
    def __init__(self):
        super().__init__('piper_conrol_gamesir')

        # --- Publishers ---
        self.pub_cart = self.create_publisher(Point, '/piper_cartesian_delta', 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_commands_joi', 10)
        self.pub_grip = self.create_publisher(Float64, '/gripper_command', 10)
        self.pub_mode = self.create_publisher(String, '/control_mode', 10)

        # --- Subscribers ---
        self.sub_mode = self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- Inicializar pygame ---
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error(" No se detectó ningún gamepad.")
            exit()
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"Gamepad detectado: {self.joy.get_name()}")

        # --- Parámetros ---
        self.joint_step = 0.05
        self.cart_step = 0.01
        self.deadzone = 0.5
        self.mode_cart = False   # iniciar en modo articular
        self.last_btn_mode = 0
        self.external_mode_override = False
        self.grip_val = 0.0

        self.current_joints = [0.0]*8
        self.joint_names = [f'joint{i+1}' for i in range(8)]

        # --- Timers ---
        self.timer = self.create_timer(0.05, self.loop)
        self.publish_mode()
        self.home_timer = self.create_timer(1.0, self._trigger_home_once)

    def _trigger_home_once(self):
        """Ejecuta el envío de home solo una vez y luego cancela el timer."""
        self.send_home_position_once()
        self.home_timer.cancel()  # 🔹 Detiene el timer, no se vuelve a llamar

    # ============================================================
    def send_home_position_once(self):
        """Envía la posición home (todos los joints y gripper en 0) una vez al inicio."""
        self.mode_cart = False
        self.publish_mode()

        msg = JointState()
        msg.name = self.joint_names
        msg.position = [0.0]*8
        msg.velocity = [0.0]*8
        msg.effort = [0.0]*8
        self.pub_joint.publish(msg)

        # Cerrar gripper también
        self.pub_grip.publish(Float64(data=0.0))

        self.get_logger().info("🏠 Enviando posición inicial (0 rad, gripper cerrado).")

    # ============================================================
    def go_home_and_exit(self):
        """Al cerrar el nodo: pasa a modo joint, envía a home y cierra gripper."""
        self.get_logger().info("🛑 Saliendo: enviando posición HOME y cerrando gripper...")

        # Cambiar a modo articular
        self.mode_cart = False
        self.publish_mode()

        # Publicar posición 0
        msg = JointState()
        msg.name = self.joint_names
        msg.position = [0.0]*8
        msg.velocity = [0.0]*8
        msg.effort = [0.0]*8
        self.pub_joint.publish(msg)

        # Cerrar gripper
        self.pub_grip.publish(Float64(data=0.0))

        # Esperar un instante para asegurar envío
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("✅ Robot en posición HOME y gripper cerrado. Cerrando nodo...")

    # ============================================================
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= len(self.current_joints):
            self.current_joints = list(msg.position[:8])

    # ============================================================
    def publish_mode(self):
        msg = String()
        msg.data = "cartesian" if self.mode_cart else "joint"
        self.pub_mode.publish(msg)

    # ============================================================
    def mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if mode == "cartesian" and not self.mode_cart:
            self.mode_cart = True
            self.external_mode_override = True
            self.get_logger().info("🛰️ Modo cambiado externamente a CARTESIANO 🟢")
        elif mode == "joint" and self.mode_cart:
            self.mode_cart = False
            self.external_mode_override = True
            self.get_logger().info("🛰️ Modo cambiado externamente a ARTICULAR 🔵")

    # ============================================================
    def loop(self):
        pygame.event.pump()
        btn_mode = self.joy.get_button(7)

        # Cambiar modo con botón
        if btn_mode and not self.last_btn_mode:
            self.mode_cart = not self.mode_cart
            modo = "cartesian" if self.mode_cart else "joint"
            self.get_logger().info(f"🔁 Cambiado a modo {modo.upper()}")
            self.publish_mode()
            self.external_mode_override = False
        self.last_btn_mode = btn_mode

        # Ejecutar modo activo
        if self.mode_cart:
            self.handle_cartesian()
        else:
            self.handle_joint_delta()

    # ============================================================
    def handle_joint_delta(self):
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hats = [self.joy.get_hat(i) for i in range(self.joy.get_numhats())]
        buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]

        dq = [0.0]*6
        if abs(axes[0]) > self.deadzone:
            dq[0] = self.joint_step * (-1 if axes[0] > 0 else 1)
        hat_x, hat_y = hats[0]
        if hat_x != 0:
            dq[1] = self.joint_step * hat_x
        if hat_y != 0:
            dq[2] = self.joint_step * hat_y
        if abs(axes[3]) > self.deadzone:
            dq[3] = self.joint_step * (-1 if axes[3] > 0 else 1)
        if buttons[3]:
            dq[4] = self.joint_step
        elif buttons[1]:
            dq[4] = -self.joint_step
        if buttons[4]:
            dq[5] = self.joint_step
        elif buttons[0]:
            dq[5] = -self.joint_step

        # Gripper (eje 5)
        grip_axis = axes[5]
        self.grip_val = 0.035 - 0.035 * (-grip_axis + 1) / 2.0

        new_positions = self.current_joints.copy()
        for i in range(6):
            new_positions[i] += dq[i]
        new_positions[6] = self.grip_val
        if len(new_positions) < 8:
            new_positions += [0.0] * (8 - len(new_positions))

        msg = JointState()
        msg.name = self.joint_names
        msg.position = new_positions
        msg.velocity = [0.0]*8
        msg.effort = [0.0]*8
        self.pub_joint.publish(msg)

        os.system('clear')
        print("[JOINT ABS MODE]")
        for i, val in enumerate(new_positions[:6], 1):
            print(f"  joint{i}: {val:+.3f}")
        print(f"  gripper: {self.grip_val:.3f}")

    # ============================================================
    def handle_cartesian(self):
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hat_x, hat_y = self.joy.get_hat(0)
        axis_x = 0
        axis_z = 3

        dx = self.cart_step * (-1 if axes[axis_x] > 0 else 1) if abs(axes[axis_x]) > self.deadzone else 0.0
        dy = self.cart_step * hat_y if hat_y != 0 else 0.0
        dz = self.cart_step * (-1 if axes[axis_z] > 0 else 1) if abs(axes[axis_z]) > self.deadzone else 0.0

        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            msg = Point(x=dx, y=dy, z=dz)
            self.pub_cart.publish(msg)
            os.system('clear')
            print(f"[CARTESIAN Δ] Δx={dx:+.3f}  Δy={dy:+.3f}  Δz={dz:+.3f}")

# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = PiperTeleopGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.go_home_and_exit()  # 👉 Envía home y cierra gripper antes de salir
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()