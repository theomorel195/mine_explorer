#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import yaml
import sys
import math

# Fonction utilitaire pour convertir un quaternion en angle de lacet (Yaw)
def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class RobotMover(Node):
    def __init__(self, trajectory_file):
        super().__init__('robot_mover')

        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        
        # S'abonner à l'odométrie
        self.odom_sub = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)

        self.trajectory = self.load_trajectory(trajectory_file)
        self.current_step_index = 0
        
        # Variables d'état
        self.initial_pose = None
        self.current_pose = None
        self.goal_reached = False

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Starting Odometry-based trajectory...')

    def load_trajectory(self, trajectory_file):
        with open(trajectory_file, 'r') as f:
            data = yaml.safe_load(f)
        return data['trajectory']

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

    def get_distance_moved(self):
        if not self.initial_pose or not self.current_pose: 
            return 0.0
        dx = self.current_pose.position.x - self.initial_pose.position.x
        dy = self.current_pose.position.y - self.initial_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        return distance

    def get_angle_turned(self):
        if not self.initial_pose or not self.current_pose: 
            return 0.0
        
        initial_yaw = euler_from_quaternion(self.initial_pose.orientation)
        current_yaw = euler_from_quaternion(self.current_pose.orientation)
        
        # Calcul de la différence d'angle la plus courte (gère le passage de -PI à PI)
        diff = current_yaw - initial_yaw
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        
        return abs(diff) # On retourne la valeur absolue pour comparer à 'duration'

    def control_loop(self):
        if self.current_step_index >= len(self.trajectory) or self.current_pose is None:
            self.publisher_.publish(Twist())
            return

        step = self.trajectory[self.current_step_index]
        linear_vel = float(step.get('linear', 0.0))
        angular_vel = float(step.get('angular', 0.0))
        
        # On définit l'objectif : si angular != 0, l'objectif est un angle, sinon une distance
        # On utilise 'duration' comme valeur cible (ex: 1.57 pour 90 degrés ou 2.0 pour 2 mètres)
        target = float(step.get('duration', 1.0))

        if angular_vel != 0:
            progress = self.get_angle_turned()
        else:
            progress = self.get_distance_moved()

        if progress < target:
            msg = Twist()
            msg.linear.x = linear_vel
            msg.angular.z = angular_vel
            self.publisher_.publish(msg)
        else:
            self.get_logger().info(f'Step {self.current_step_index + 1} completed via Odom.')
            self.current_step_index += 1
            self.initial_pose = self.current_pose # Reset pour la prochaine étape

def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print('Usage: robot_mover <trajectory_yaml_path>')
        return

    trajectory_path = sys.argv[1]

    node = RobotMover(trajectory_path)

    try:
        # C'est ici que la magie opère : spin() permet au nœud de fonctionner
        # sans bloquer les communications.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Arrêt de sécurité
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()