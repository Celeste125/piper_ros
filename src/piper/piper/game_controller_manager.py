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
        super().__init__('piper_teleop_gamepad')
        # --- Publishers ---
        self.pub_cart = self.create_publisher(Point, '/piper_cartesian_delta', 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_commands_joi', 10)
        self.pub_grip = self.create_publisher(Float64, '/gripper_command', 10)
        self.pub_mode = self.create_publisher(String, '/control_mode', 10)
        # --- Subscriber para modo externo ---
        self.sub_mode = self.create_subscription(String, '/control_mode', self.mode_callback, 10)
        # --- Initializes pygame ---
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error(":x: No gamepad detected.")
            exit()
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f":white_check_mark: Gamepad detected: {self.joy.get_name()}")
        # --- Parámetros ---
        self.joint_step = 0.05     # rad (incremento)
        self.cart_step = 0.01      # m (incremento)
        self.deadzone = 0.5
        self.mode_cart = True      # modo inicial cartesiano
        self.last_btn_mode = 0
        self.external_mode_override = False
        self.gripper_effort = 1.0
        self.grip_val = 0.0        # valor absoluto del gripper
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        # Publicar modo inicial
        self.publish_mode()
    # ============================================================
    def publish_mode(self):
        """Publishes the current mode to /control_mode"""
        msg = String()
        msg.data = "cartesian" if self.mode_cart else "joint"
        self.pub_mode.publish(msg)
    # ============================================================
    def mode_callback(self, msg: String):
        """Receives external mode ('cartesian' or 'joint')"""
        mode = msg.data.strip().lower()
        if mode == "cartesian" and not self.mode_cart:
            self.mode_cart = True
            self.external_mode_override = True
            self.get_logger().info(":satellite: Mode externally changed to CARTESIAN :large_green_circle:")
        elif mode == "joint" and self.mode_cart:
            self.mode_cart = False
            self.external_mode_override = True
            self.get_logger().info(":satellite: MMode externally changed to JOINT :large_blue_circle:")
    # ============================================================
    def loop(self):
        pygame.event.pump()
        btn_mode = self.joy.get_button(7)
        # Change mode with button and publish it
        if btn_mode and not self.last_btn_mode:
            self.mode_cart = not self.mode_cart
            modo = "cartesian" if self.mode_cart else "joint"
            self.get_logger().info(f":repeat: Switched to {modo.upper()}")
            self.publish_mode()
            self.external_mode_override = False
        self.last_btn_mode = btn_mode
        # Execute active mode
        if self.mode_cart:
            self.handle_cartesian()
        else:
            self.handle_joint_delta()
    # ============================================================
    def handle_joint_delta(self):
        """Joint incremental mode: publishes Δjoint (rad) + absolute gripper"""
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hats = [self.joy.get_hat(i) for i in range(self.joy.get_numhats())]
        buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        dq = [0.0]*6  # Incrementos Δq₁…Δq₆
        # ---- Joint 1: axis 0 ----
        if abs(axes[0]) > self.deadzone:
            dq[0] = self.joint_step * (-1 if axes[0] > 0 else 1)
        # ---- Joint 2: hat x ----
        hat_x, hat_y = hats[0]
        if hat_x != 0:
            dq[1] = self.joint_step * hat_x
        # ---- Joint 3: hat y ----
        if hat_y != 0:
            dq[2] = self.joint_step * hat_y
        # ---- Joint 4: eje 3 ----
        if abs(axes[3]) > self.deadzone:
            dq[3] = self.joint_step * (-1 if axes[3] > 0 else 1)
        # ---- Joint 5: botones 3 (+), 0 (–) ----
        if buttons[3]:
            dq[4] = self.joint_step
        elif buttons[1]:
            dq[4] = -self.joint_step
        # ---- Joint 6: botones 1 (+), 2 (–) ----
        if buttons[4]:
            dq[5] = self.joint_step
        elif buttons[0]:
            dq[5] = -self.joint_step
        # ---- Gripper (joint7): eje 5 ----
        grip_axis = axes[5]
        # Valor absoluto 0.0–0.35
        self.grip_val = 0.35 * (-grip_axis + 1) / 2.0 if grip_axis <= 0 else 0.0
        self.pub_grip.publish(Float64(data=self.grip_val))
        # --- Armar mensaje JointState ---
        msg = JointState()
        msg.name = [f'joint{i+1}' for i in range(8)]  # joint1..joint8
        msg.position = dq + [self.grip_val, 0.0]      # Δq₁–₆ + gripper + dummy
        msg.velocity = [0.0]*8
        msg.effort = [0.0]*8
        self.pub_joint.publish(msg)
        os.system('clear')
        print("[JOINT Δ MODE]")
        for i, val in enumerate(dq, 1):
            print(f"  Δjoint{i}: {val:+.3f}")
        print(f"  joint7 (gripper): {self.grip_val:.3f}")
        print(f"  joint8: 0.000")
    # ============================================================
    def handle_cartesian(self):
        """Crtesian mode: publishes only increments (Δx, Δy, Δz)"""
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hat_x, hat_y = self.joy.get_hat(0)
        # --- Ejes ---
        axis_x = 0  # joystick izquierdo horizontal
        axis_y = 1  # joystick  
        axis_z = 3  # joystick derecho vertical (ajusta si tu mando usa otro)
        dx = self.cart_step * (-1 if axes[axis_x] > 0 else 1) if abs(axes[axis_x]) > self.deadzone else 0.0
        #dy = self.cart_step * hat_y if hat_y != 0 else 0.0
        dy = self.cart_step * (-1 if axes[axis_y] > 0 else 1) if abs(axes[axis_y]) > self.deadzone else 0.0
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
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
