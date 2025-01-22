#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import math
import threading 
import time
num_robots = 4

# List of positions of all robots
all_positions = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}
# List of velocities of all robots
all_velocities = {i: np.array([0.0, 0.0]) for i in range(num_robots)}

 
class ConcensusController(Node):
    def __init__(self, num_robots, max_vel=2, adj_matrix=None, formation=None):
        super().__init__('concensus_controller')

        self.num_robots = num_robots
        self.adj_matrix = adj_matrix
        self.formation = formation

        # Maximum velocity
        self.max_vel = max_vel

        self.formation_tau = 0
 
        # Publishers for robot velocity commands
        self.cmd_vel_publishers = [
            self.create_publisher(Twist, f'cf_{robot_id +1}/cmd_vel', 10) for robot_id in range(num_robots)
            
        ]
        # Podatci o mapi
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        for boid_id in range(1, num_robots + 1):
            self.create_subscription(Odometry, f'cf_{boid_id}/odom', lambda msg, robot_id=boid_id-1: self.odom_callback(msg, robot_id), 10)
            self.create_subscription(Twist, f'cf_{boid_id}/cmd_vel', lambda msg, robot_id=boid_id-1: self.cmd_vel_callback(msg, robot_id), 10)


        self.create_subscription(Point, '/leader_goal', self.leader_goal_callback, 10)
        self.leader_goal = np.array([0.0, 0.0])

    def leader_goal_callback(self, msg):
        """Update migration goal with the received coordinates."""
        self.leader_goal = np.array([msg.x, msg.y])

    def cmd_vel_callback(self, msg, robot_id=None):
        """Save velocity information for robots."""
        vel = np.array([msg.linear.x, msg.linear.y])
        all_velocities[robot_id] = vel

    def map_callback(self, msg):
        """Save map information."""
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def odom_callback(self, msg, robot_id=None):
        """Save velocity and position information about other robots."""
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        all_positions[robot_id] = pos
        all_velocities[robot_id] = vel

    def separation_acel(self, neighborhood_distance=1.0, neighborhood_angle=360, self_id=None):
        """Calculate velocity to keep separate from neighbours."""
        separation_force = np.array([0.0, 0.0])
        for i, pos in all_positions.items():
            if i != self_id:
                pos_diff = pos - all_positions[self_id]
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance / 1.8:
                    pos_diff_normalized = pos_diff / distance
                    velocity_normalized = all_velocities[self_id] / (np.linalg.norm(all_velocities[self_id]) + 0.00001)

                    dot_product = np.dot(velocity_normalized, pos_diff_normalized)
                    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Angle in radians

                    angle_degrees = np.degrees(angle)
                    if angle_degrees <= int(neighborhood_angle) / 2:
                        separation_force += (all_positions[self_id] - pos) / distance ** 1.5
        return separation_force

    def concensus(self, self_id=None, alpha=1.0 / 50):
        """Consensus algorithm for controlling the robots."""
        
        desired_velocity = np.array([0.0, 0.0])

        sepparation_acceleration = self.separation_acel(1, 320, self_id=self_id)

        desired_velocity += sepparation_acceleration * alpha
        if self_id == 0:
            
            acceleration_to_goal = self.leader_goal - all_positions[self_id]
            distance_to_goal = np.linalg.norm(acceleration_to_goal)

            desired_velocity = all_velocities[self_id] + acceleration_to_goal * alpha
            
        
            if np.linalg.norm(desired_velocity) > self.max_vel:
                desired_velocity = desired_velocity / (np.linalg.norm(desired_velocity)  + 0.00001)* self.max_vel

            if distance_to_goal < 1:
                desired_velocity = desired_velocity * np.tanh(1.5 * distance_to_goal)
        else:
            temp_formation = self.formation + self.formation_tau
            for robot_id in range(self.num_robots):
                desired_velocity += (
                    self.adj_matrix[self_id][robot_id] * (all_positions[robot_id] - all_positions[self_id])
                    - (temp_formation[robot_id] - temp_formation[self_id])
                )
            if np.linalg.norm(desired_velocity) > self.max_vel:
                desired_velocity = desired_velocity / (np.linalg.norm(desired_velocity)  + 0.00001)* self.max_vel



        
        twist = Twist()


        twist.linear.x = desired_velocity[0]
        twist.linear.y = desired_velocity[1]
        twist.angular.z = 0.0
        self.cmd_vel_publishers[self_id].publish(twist)
        all_velocities[self_id] = desired_velocity
    


def spin_in_thread(node):
    """Helper function to spin the node in a separate thread."""
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)

    max_vel = 0.5

    if(num_robots == 3): 
        adj_matrix = np.array([[0, 1, 1],
                           [1, 0, 1],
                           [1, 1, 0]])

        formation_square = np.array([[0.,0.],
                    [0., 1.],
                    [1., 1.]])
        formation_line = np.array([[0.,0.],
                    [0., 1.],
                    [0., 2.]])
        
        formation_ltriangle = np.array([[0.,0.],
                    [2., 0.],
                    [1., 1.]])
    else:

        adj_matrix_fully_connected = np.array([[0, 1, 1, 1],
                            [1, 0, 1, 1],
                            [1, 1, 0, 1],
                            [1, 1, 1, 0]])

        adj_matrix_square= np.array([[0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1],
                            [1, 0, 0, 0]])

        formation_square = np.array([
        [0., 0.],  # Bottom-left corner
        [0., 1.],  # Top-left corner
        [1., 1.],  # Top-right corner
        [1., 0.]   # Bottom-right corner
        ])

        formation_line = np.array([
        [0., 0.],  # First drone
        [0., 1.],  # Second drone
        [0., 2.],  # Third drone
        [0., 3.]   # Fourth drone
        ])

        formation_triangle = np.array([
        [0., 0.],    # Bottom-left corner
        [2., 0.],    # Bottom-right corner
        [1., np.sqrt(3.)],  # Top corner (height of the triangle)
        [1., np.sqrt(3.) / 3.]  # Centroid of the triangle
        ])

    controller = ConcensusController(num_robots=num_robots, max_vel=max_vel, adj_matrix=adj_matrix_square, formation=formation_square)

    spin_thread_instance = threading.Thread(target=spin_in_thread, args=(controller,))
    spin_thread_instance.daemon = True 
    spin_thread_instance.start()

    alpha = 50
    rate = controller.create_rate(alpha)

    try:
        while rclpy.ok():
            for robot_id in range(num_robots):
                controller.concensus(self_id=robot_id, alpha = alpha)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        spin_thread_instance.join() 


if __name__ == '__main__':
    main()
