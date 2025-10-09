#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PositionConstraint, Constraints
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

class SimplePoseGoal(Node):
    def __init__(self):
        super().__init__('simple_pose_goal')
        
        # Action client para MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        print("=== SIMPLE POSE GOAL ===")
        
        # Esperar por action server
        if self._action_client.wait_for_server(timeout_sec=5.0):
            print("✅ Action server listo!")
            self.send_pose_goal()
        else:
            print("❌ Action server no disponible")
            rclpy.shutdown()
    
    def send_pose_goal(self):
        """Envía una pose goal específica (similar al ejemplo C++)"""
        
        # Definir la pose objetivo (igual que en el ejemplo)
        target_pose = Pose()
        target_pose.orientation.w = 1.0
        target_pose.position.x = 0.28
        target_pose.position.y = -0.2
        target_pose.position.z = 0.5
        
        print(f"🎯 Enviando pose goal:")
        print(f"   📍 Posición: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")
        print(f"   🧭 Orientación: w={target_pose.orientation.w}")
        
        # Crear el goal
        goal = MoveGroup.Goal()
        
        # Configurar la solicitud de planificación
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.pipeline_id = "ompl"
        
        # Configurar constraints de posición cartesiana
        constraints = Constraints()
        
        # Position constraint para el end effector
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "end_effector_link"  # Ajusta según tu robot
        
        # Definir región permitida
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerancia
        
        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(target_pose)
        position_constraint.weight = 1.0
        
        constraints.position_constraints.append(position_constraint)
        goal.request.goal_constraints.append(constraints)
        
        # Opciones de planificación (Plan + Execute)
        goal.planning_options.plan_only = False  # ¡IMPORTANTE: Ejecutar después de planificar!
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.replan_delay = 2.0
        
        # Enviar goal
        print("🚀 Enviando Plan & Execute...")
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Callback cuando el action server responde"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                print("❌ Goal rechazado por el action server")
                rclpy.shutdown()
                return
            
            print("✅ Goal aceptado! Planificando y ejecutando...")
            
            # Obtener resultado
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._get_result_callback)
            
        except Exception as e:
            print(f"❌ Error en goal: {e}")
            rclpy.shutdown()
    
    def _get_result_callback(self, future):
        """Callback cuando se completa la acción"""
        try:
            result = future.result().result
            error_code = result.error_code.val
            
            if error_code == 1:  # SUCCESS
                print("🎉 ¡MOVIMIENTO EXITOSO! El robot debería haberse movido.")
            else:
                print(f"❌ Error en movimiento: código {error_code}")
                
        except Exception as e:
            print(f"❌ Error en resultado: {e}")
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SimplePoseGoal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()