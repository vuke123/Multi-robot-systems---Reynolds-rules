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
    def __init__(self,  num_robots, separation_weight=0.2, max_vel = 2, adj_matrix = None):
        super().__init__('concensus_controller')


        self.num_robots = num_robots

        self.adj_matrix = adj_matrix
	    
	    # Težine pridodane pravilima
        self.separation_weight = separation_weight
        
        # Maksimalna brzina
        self.max_vel = max_vel
        
        self.cmd_vel_publishers = [
            self.create_publisher(Twist, f'cf_{robot_id +1}/cmd_vel', 10) for robot_id in range(num_robots)
            
        ]

        time.sleep(2)

        for boid_id in range(1, num_robots + 1):
            self.create_subscription(Odometry, f'cf_{boid_id}/odom', lambda msg, robot_id=boid_id-1: self.odom_callback(msg, robot_id), 10)
            self.create_subscription(Twist, f'cf_{boid_id}/cmd_vel', lambda msg, robot_id=boid_id-1: self.cmd_vel_callback(msg, robot_id), 10)


    def cmd_vel_callback(self, msg, robot_id = None):
        vel = np.array([msg.linear.x, msg.linear.y])

        all_velocities[robot_id] = vel

        
    def odom_callback(self, msg, robot_id = None):
        """Save velocity and position information about other boids."""
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
                        separation_force += (all_positions[self_id] - pos) / distance ** 2
        return separation_force


    def concensus(self, self_id=None, alpha=1.0 / 50):

        sepparation_acceleration = self.separation_acel(1, 320, self_id=self_id)

        difference_sum = np.array([0.0, 0.0])
        for robot_id in range(self.num_robots):
            difference_sum += self.adj_matrix[self_id][robot_id]*(all_positions[robot_id] - all_positions[self_id])
        desired_position = all_positions[self_id] + alpha * difference_sum
        

        direction_to_goal = desired_position - all_positions[self_id]
        distance_to_goal = np.linalg.norm(direction_to_goal)
        concensus_accel = direction_to_goal / distance_to_goal


        ssepparation_acceleration = self.separation_acel(1, 320, self_id=self_id)

        separation_velocity = sepparation_acceleration * alpha 

        twist = Twist() 


        desired_velocity = all_velocities[self_id] + separation_velocity * self.separation_weight  + concensus_accel * (1 - self.separation_weight)
        
        if np.linalg.norm(desired_velocity) > self.max_vel: # Clippanje brzine prema potrebama zadatka/robota
            desired_velocity = desired_velocity / np.linalg.norm(desired_velocity)  * self.max_vel

        
        dist_to_center_of_mass = np.linalg.norm((difference_sum / (self.num_robots - 1)) - all_positions[self_id])
        if dist_to_center_of_mass < 1: # Arrival s tahn gušenjem
            desired_velocity = desired_velocity * np.tanh(1.5 * dist_to_center_of_mass)

      
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
    neighborhood_distance = 1

    adj_matrix_fully_connected= np.array([[0, 1, 1, 1],
                            [1, 0, 1, 1],
                            [1, 1, 0, 1],
                            [1, 1, 1, 0]])
    
    adj_matrix_square= np.array([[0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1],
                            [1, 0, 0, 0]])

    adj_matrix = np.array([[0, 1, 1],
                           [1, 0, 1],
                           [1, 1, 0]])          

    controller = ConcensusController(num_robots=num_robots, max_vel = max_vel, adj_matrix = adj_matrix_fully_connected, separation_weight = 0.7) 
    

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
