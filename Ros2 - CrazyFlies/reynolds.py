#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import math
import time
import threading

num_robots = 4

# Lista pozicija svih robota
all_positions = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}
# Lista brzina svih robota
all_velocities = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}
all_velocities = {i: np.array([0.5, 0.5]) for i in range(num_robots)}

def spin_in_thread(controller):
    """Function to spin the ROS2 node in a separate thread."""
    rclpy.spin(controller)

class ReynoldsRulesController(Node):
    def __init__(self,  num_robots, dodge_weight = 0.8, separation_weight=0.6, alignment_weight=0.8, cohesion_weight=0.6, migration_weight = 1.2, migration_urge = True, max_vel = 3):
        super().__init__('reynolds_controller')


        
        self.num_robots = num_robots
        
        # Težine pridodane pravilima
        self.weights = [dodge_weight, separation_weight, alignment_weight, cohesion_weight, migration_weight]
        # Ima li simulaciju migraciju, ili ne
        self.migration_urge = migration_urge
        # Maksimalna brzina
        self.max_vel = max_vel
        
        # Migration goal of the flock na početku 
        self.migration_goal = np.array([0.0, 0.0])
        
        self.cmd_vel_publishers = [self.create_publisher(Twist, f"cf_{robot_id+1}/cmd_vel", 10) for robot_id in range(num_robots)]
        self.goal_sub = self.create_subscription(Point, '/migration_goal', self.migration_goal_callback, 10)

        # Početna brzina
        for robot_id in range(num_robots):
            vel = np.array([0.5, 0.5]) * self.max_vel 
            twist = Twist()
            twist.linear.x = vel[0]
            twist.linear.y = vel[1]
            twist.angular.z = 0.0  
            self.cmd_vel_publishers[robot_id].publish(twist)
        

        # Podatci o mapi
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        
        time.sleep(2)

        for boid_id in range(num_robots):
            self.create_subscription(Odometry, f'cf_{boid_id+1}/odom', lambda msg, robot_id=boid_id: self.odom_callback(msg, robot_id), 10)

        
    def map_callback(self, msg): 
        """
        Save map information to boid.
        """
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
    def get_nearest_obstacle_in_front(self, range_ahead=0.4, num_rays=19, dodge_angle = np.pi / 4, robot_id = None):
        """
        Get nearest obstacle in front of boid, at the maximum range of "range_ahead". 
        The object search is performed in the direction of num_rays rays in a cone
        pointed in fron of the boid with the angele of dodge_angle*2
        """
        if self.map_data is None:
            return None
        map_x = int((all_positions[robot_id][0] - self.map_origin[0]) / self.map_resolution)
        map_y = int((all_positions[robot_id][1] - self.map_origin[1]) / self.map_resolution)
        
        velocity_angle = np.arctan2(all_velocities[robot_id][1], all_velocities[robot_id][0])
        angles = np.linspace(velocity_angle - dodge_angle , velocity_angle + dodge_angle , num_rays)  # -30 0 +30
        
        obstacle_positions = []
        for angle in angles:
            nearest_obstacle_distance = float('inf')  
            for i in range(1, int(range_ahead / self.map_resolution)):  
                x_offset = i * math.cos(angle) * self.map_resolution
                y_offset = i * math.sin(angle) * self.map_resolution
                ray_x = int(map_x + x_offset / self.map_resolution)
                ray_y = int(map_y + y_offset / self.map_resolution)
                if 0 <= ray_x < self.map_data.shape[1] and 0 <= ray_y < self.map_data.shape[0]:
                    if self.map_data[ray_y, ray_x] == 100:
                        nearest_obstacle_distance = min(nearest_obstacle_distance, i * self.map_resolution)
                        min_ray_y = ray_y
                        min_ray_x = ray_x
                        break  
		    
            if nearest_obstacle_distance != float('inf'):
                obstacle_real_x = self.map_origin[0] + min_ray_x * self.map_resolution
                obstacle_real_y = self.map_origin[1] + min_ray_y * self.map_resolution
                obstacle_positions.append(np.array([obstacle_real_x, obstacle_real_y]))
            else:
                obstacle_positions.append(None)
        
        return obstacle_positions
        
        
    def object_dodge_vel(self, neighborhood_distance = 2.0, neighborhood_angle = 180,num_rays = 31, dodge_angle = np.pi / 4, robot_id = None):
        """
        Calculate the velocity to avoid detected objects in neighbourhood_distance range.
        """
        obstacle_positions = self.get_nearest_obstacle_in_front(num_rays = num_rays, dodge_angle = dodge_angle, robot_id = robot_id)

        if obstacle_positions is None:
            return np.array([0.0, 0.0])
        dodge_force = np.array([0.0, 0.0])
        
        for obstacle_position in obstacle_positions:
            if obstacle_position is not None:
                pos_diff = obstacle_position - all_positions[robot_id]
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance : 
                    dodge_force += (all_positions[robot_id] - obstacle_position) / distance**1.6

        return dodge_force
        
    def odom_callback(self, msg, robot_id = None):
        """Save velocity and position information about other boids."""
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        all_positions[robot_id] = pos
        all_velocities[robot_id] = vel

            
            
    def cmd_vel_callback(self, msg, robot_id = None):
        vel = np.array([msg.linear.x, msg.linear.y])

        all_velocities[robot_id] = vel


    def alignment_vel(self, neighborhood_distance=1.0, neighborhood_angle = 300, self_id = None):
        """Calculate velocity to better align with neighbours."""
        average_velocity = np.array([0.0, 0.0])
        count = 0
        # Racunanje smjera u kojem boid gleda, normalizirani vektor brzine
        velocity_normalized = all_velocities[self_id] / (np.linalg.norm(all_velocities[self_id]) + 0.001)

        for i, (pos, vel) in enumerate(zip(all_positions.values(), all_velocities.values())):
            if i != self_id:  
                pos_diff = pos - all_positions[self_id]
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance :
                    # Normaliziraj vektor razlike pozicija
                    pos_diff_normalized = pos_diff / distance

                    # Normaliziraj vektor brzine (vektor gledanja)
                    velocity_normalized = all_velocities[self_id] / (np.linalg.norm(all_velocities[self_id]) + 0.001)

                    # Izračunaj kut između vektora brzine i vektora razlike pozicija
                    dot_product = np.dot(velocity_normalized, pos_diff_normalized)
                    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Kut u radijanima

                    # Provjeri je li kut unutar vidnog polja
                    angle_degrees = np.degrees(angle)
                    if angle_degrees <= int(neighborhood_angle) / 2:  # Polovina vidnog polja na svaku stranu
                        average_velocity += vel
                        count += 1
        if count > 0: # Računaj prosjek samo ako imaš susjeda
            average_velocity /= count
            return (average_velocity - all_velocities[self_id])
        else: #Ako nema susjeda, nema ni aligmenta
            return np.array([0.0, 0.0])


    def cohesion_vel(self, neighborhood_distance=1.0, neighborhood_angle = 300, self_id = None):
        """Calculate velocity to position at the center of mass of boid neighbourhood."""
        center_of_mass = np.array([0.0, 0.0])
        count = 0
        for i, pos in all_positions.items():
            if i != self_id:  
                pos_diff = pos - all_positions[self_id]
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance :
                    # Normaliziraj vektor razlike pozicija
                    pos_diff_normalized = pos_diff / distance

                    # Normaliziraj vektor brzine (vektor gledanja)
                    velocity_normalized = all_velocities[self_id] / (np.linalg.norm(all_velocities[self_id]) + 0.001)

                    # Izračunaj kut između vektora brzine i vektora razlike pozicija
                    dot_product = np.dot(velocity_normalized, pos_diff_normalized)
                    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Kut u radijanima

                    # Provjeri je li kut unutar vidnog polja
                    angle_degrees = np.degrees(angle)

                    if angle_degrees <= int(neighborhood_angle) / 2:  # Polovina vidnog polja na svaku stranu
                        center_of_mass += pos
                        count += 1
                        
        if count > 0:
            center_of_mass /= count
            return (center_of_mass - all_positions[self_id]  ) 
        else: #Ako nema susjeda, nema kohezije
            return np.array([0.0, 0.0])
        

    def migration_goal_callback(self, msg):
        # Update migration goal with the received coordinates
        self.migration_goal = np.array([msg.x, msg.y])

    def migration_goal_vel(self, self_id = None):
        # Compute velocity towards the migration goal
        direction_to_goal = self.migration_goal - all_positions[self_id]

        distance_to_goal = np.linalg.norm(direction_to_goal)
        if distance_to_goal > 0.1:
            return direction_to_goal / distance_to_goal**1.5, distance_to_goal # Normalize direction
        return np.array([0.0, 0.0]), distance_to_goal

    def separation_vel(self, neighborhood_distance=1.0, neighborhood_angle = 300, sep_exp = 1, self_id = None):
        """Calculate velocity to keep separate from neighbours."""
        separation_force = np.array([0.0, 0.0])
        for i, pos in all_positions.items():
            if i != self_id: 
                pos_diff = pos - all_positions[self_id]
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance / 2  : 
                    # Normaliziraj vektor razlike pozicija
                    pos_diff_normalized = pos_diff / distance

                    # Normaliziraj vektor brzine (vektor gledanja)
                    velocity_normalized = all_velocities[self_id] / (np.linalg.norm(all_velocities[self_id]) + 0.001)

                    # Izračunaj kut između vektora brzine i vektora razlike pozicija
                    dot_product = np.dot(velocity_normalized, pos_diff_normalized)
                    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Kut u radijanima

                    # Provjeri je li kut unutar vidnog polja
                    angle_degrees = np.degrees(angle)
                    if angle_degrees <= int(neighborhood_angle) / 2:  # Polovina vidnog polja na svaku stranu
                        separation_force += (all_positions[self_id] - pos) / distance**1.2
        return separation_force



    def compute_vel(self, neighborhood_distance=1, neighborhood_angle=300, sep_exp=1, self_id=None, max_vel = 0.2):
        """Aggregate implemented Reynolds rules, obstacle avoidance rules and migration into one velocity request."""
        

        
        if not self.migration_urge:
            self.weights[-1] = 0
        velocities = []
        velocities.append(self.object_dodge_vel(2, neighborhood_angle, robot_id=self_id))
        velocities.append(self.separation_vel(neighborhood_distance, neighborhood_angle, sep_exp=sep_exp, self_id=self_id))
        velocities.append(self.alignment_vel(neighborhood_distance, neighborhood_angle, self_id=self_id))
        velocities.append(self.cohesion_vel(neighborhood_distance, neighborhood_angle, self_id=self_id))

        mig_vel, mig_dist = self.migration_goal_vel(self_id)

        velocities.append(mig_vel)

        desired_acceleration = np.array([0.0, 0.0])


        for vel, weight in zip(velocities, self.weights): 
            desired_acceleration += vel * weight
        
        twist = Twist()
        desired_velocity = desired_acceleration * 1./50
        desired_velocity = all_velocities[self_id] + desired_velocity
        
        if np.linalg.norm(desired_velocity) > self.max_vel:
            desired_velocity = desired_velocity / np.linalg.norm(desired_velocity)  * self.max_vel
            
        if self.migration_urge and mig_dist < 1: 
            desired_velocity = desired_velocity * (np.tanh(1.5* mig_dist))
        elif np.linalg.norm(desired_velocity) < 0.3*max_vel:
            desired_velocity = desired_velocity + desired_velocity / (np.linalg.norm(desired_velocity) + 0.001) * 0.1
        
            
        twist.linear.x = desired_velocity[0]
        twist.linear.y = desired_velocity[1]
        twist.angular.z = 0.0  
        self.cmd_vel_publishers[self_id].publish(twist)
        all_velocities[self_id] = desired_velocity


def main():
    rclpy.init()
    max_vel = 0.5
    neighborhood_distance = 1.5

    controller = ReynoldsRulesController(num_robots=num_robots, migration_urge=True, max_vel=max_vel) 
    rate = controller.create_rate(50)  

    spin_thread = threading.Thread(target=spin_in_thread, args=(controller,))
    spin_thread.daemon = True
    spin_thread.start()
    
    while rclpy.ok():
        for robot_id in range(num_robots):  
            controller.compute_vel(1, 320, sep_exp=1.6, self_id=robot_id, max_vel = max_vel)
        rate.sleep() 

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
