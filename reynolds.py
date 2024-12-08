#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import math
import pygame

	
class ReynoldsRules:
    def __init__(self, robot_id, num_robots, dodge_weight = 0.8, separation_weight=1., alignment_weight=0.7, cohesion_weight=0.1, migration_weight = 1, migration_urge = False, max_vel = 3):
        self.robot_id = robot_id
        self.num_robots = num_robots
	    
	    # Težine pridodane pravilima
        self.weights = [dodge_weight, separation_weight, alignment_weight, cohesion_weight, migration_weight]
        # Ima li simulaciju migraciju, ili ne
        self.migration_urge = migration_urge
        # Maksimalna brzina
        self.max_vel = max_vel
        
        # Pozicija individualnog robota
        self.position = np.array([0.0, 0.0])
			
        # Lista pozicija svih robota
        self.all_positions = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}
        # Lista brzina svih robota
        self.all_velocities = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}
		
        # Migration goal of the flock na početku 
        self.migration_goal = np.array([0.0, 0.0])


        self.odom_sub = rospy.Subscriber("robot_" +str(robot_id) + "/odom", Odometry, self.odom_callback, callback_args=robot_id)

        for boid_id in range(num_robots):
            if boid_id != robot_id:
                rospy.Subscriber("robot_" + str(boid_id) + "/odom", Odometry, self.odom_callback, callback_args=boid_id)
                rospy.Subscriber(f"robot_{boid_id}/cmd_vel", Twist, self.cmd_vel_callback, callback_args=boid_id)
        
        
        self.cmd_vel_pub = rospy.Publisher(f"robot_{robot_id}/cmd_vel", Twist, queue_size=10)

        self.goal_sub = rospy.Subscriber('/migration_goal', Point, self.migration_goal_callback)
        
        # Početna brzina
        vel = np.array([math.cos(robot_id / num_robots * 2 * math.pi), math.sin(robot_id / num_robots * 2 * math.pi)])* self.max_vel 
        
        twist = Twist()
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.angular.z = 0.0  
        self.cmd_vel_pub.publish(twist)
        self.velocity = vel
        
        # Podatci o mapi
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        
    def map_callback(self, msg): 
        """
        Save map information to boid.
        """
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
    def get_nearest_obstacle_in_front(self, range_ahead=5.0, num_rays=19, dodge_angle = np.pi / 6):
        """
        Get nearest obstacle in front of boid, at the maximum range of "range_ahead". 
        The object search is performed in the direction of num_rays rays in a cone
        pointed in fron of the boid with the angele of dodge_angle*2
        """
        if self.map_data is None:
            return None
        
        map_x = int((self.position[0] - self.map_origin[0]) / self.map_resolution)
        map_y = int((self.position[1] - self.map_origin[1]) / self.map_resolution)
        
        velocity_angle = np.arctan2(self.velocity[1], self.velocity[0])
        angles = np.linspace(velocity_angle - dodge_angle , velocity_angle + dodge_angle , num_rays)  # -30 0 +30
        
        obstacle_distances = []
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
                        break  
		    
            if nearest_obstacle_distance != float('inf'):
                obstacle_distances.append(nearest_obstacle_distance)
            else:
                obstacle_distances.append(range_ahead)

        return np.array(obstacle_distances)
        
        
    def object_dodge_vel(self, neighborhood_distance = 2.0, neighborhood_angle = 180,num_rays = 31, dodge_angle = np.pi / 6):
        """
        Calculate the velocity to avoid detected objects in neighbourhood_distance range.
        """
        obstacle_distances = self.get_nearest_obstacle_in_front(num_rays = num_rays, dodge_angle = dodge_angle)
        
        dodge_force = np.array([0.0, 0.0])
        if obstacle_distances[num_rays // 2] < neighborhood_distance:
        
            if np.sum(obstacle_distances[0: num_rays//2]) > np.sum(obstacle_distances[num_rays//2:]):
                dodge_force += np.matmul(np.array([[0, 1], [-1, 0]]), self.velocity)
            else: 
                dodge_force += np.matmul(np.array([[0, -1], [1, 0]]), self.velocity)
            dodge_force -= self.velocity*0.7
            dodge_force = dodge_force / obstacle_distances[num_rays // 2]**3
        
            
        return dodge_force
        
    def odom_callback(self, msg, robot_id):
        """Save velocity and position information about other boids."""
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        self.all_positions[robot_id] = pos
        self.all_velocities[robot_id] = vel

        if robot_id == self.robot_id:
            self.position = pos
            
            
    def cmd_vel_callback(self, msg, robot_id):
        vel = np.array([msg.linear.x, msg.linear.y])

        if robot_id != self.robot_id:
            self.all_velocities[robot_id] = vel


    def alignment_vel(self, neighborhood_distance=1.0, neighborhood_angle = 300):
        """Calculate velocity to better align with neighbours."""
        average_velocity = np.array([0.0, 0.0])
        count = 0
        # Racunanje smjera u kojem boid gleda, normalizirani vektor brzine
        velocity_normalized = self.velocity / np.linalg.norm(self.velocity)

        for i, (pos, vel) in enumerate(zip(self.all_positions.values(), self.all_velocities.values())):
            if i != self.robot_id:  
                pos_diff = pos - self.position;
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance :
                    # Normaliziraj vektor razlike pozicija
                    pos_diff_normalized = pos_diff / distance

                    # Normaliziraj vektor brzine (vektor gledanja)
                    velocity_normalized = self.velocity / np.linalg.norm(self.velocity)

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
            return (average_velocity - self.velocity)
        else: #Ako nema susjeda, nema ni aligmenta
            return np.array([0.0, 0.0])


    def cohesion_vel(self, neighborhood_distance=1.0, neighborhood_angle = 300):
        """Calculate velocity to position at the center of mass of boid neighbourhood."""
        center_of_mass = np.array([0.0, 0.0])
        count = 0
        for i, pos in self.all_positions.items():
            if i != self.robot_id:  
                pos_diff = pos - self.position;
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance :
                    # Normaliziraj vektor razlike pozicija
                    pos_diff_normalized = pos_diff / distance

                    # Normaliziraj vektor brzine (vektor gledanja)
                    velocity_normalized = self.velocity / np.linalg.norm(self.velocity)

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
            return (center_of_mass - self.position  ) 
        else: #Ako nema susjeda, nema kohezije
            return np.array([0.0, 0.0])
        

    def migration_goal_callback(self, msg):
        # Update migration goal with the received coordinates
        self.migration_goal = np.array([msg.x, msg.y])

    def migration_goal_vel(self):
        # Compute velocity towards the migration goal
        direction_to_goal = self.migration_goal - self.position
        distance_to_goal = np.linalg.norm(direction_to_goal)
        if distance_to_goal > 0.1:
            return direction_to_goal / distance_to_goal, distance_to_goal # Normalize direction
        return np.array([0.0, 0.0]), distance_to_goal

    def separation_vel(self, neighborhood_distance=1.0, neighborhood_angle = 300, sep_exp = 1):
        """Calculate velocity to keep separate from neighbours."""
        separation_force = np.array([0.0, 0.0])
        for i, pos in self.all_positions.items():
            if i != self.robot_id: 
                pos_diff = pos - self.position;
                distance = np.linalg.norm(pos_diff)

                if distance < neighborhood_distance / 1.7  : # NISAM ZIHER ZAS JE NAOMI STAVILA /2 distance ovdje
                    # Normaliziraj vektor razlike pozicija
                    pos_diff_normalized = pos_diff / distance

                    # Normaliziraj vektor brzine (vektor gledanja)
                    velocity_normalized = self.velocity / np.linalg.norm(self.velocity)

                    # Izračunaj kut između vektora brzine i vektora razlike pozicija
                    dot_product = np.dot(velocity_normalized, pos_diff_normalized)
                    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Kut u radijanima

                    # Provjeri je li kut unutar vidnog polja
                    angle_degrees = np.degrees(angle)
                    if angle_degrees <= int(neighborhood_angle) / 2:  # Polovina vidnog polja na svaku stranu
                        separation_force += (self.position - pos) / distance**sep_exp
        return separation_force


    def compute_vel(self, neighborhood_distance=1, neighborhood_angle = 300, sep_exp = 1):
        """Agregate implemented reynolds rules, obstacle avoidance rules and migration into one velocity request."""
        
        # Mode
        mode = "weighted"
        max_speed = 1
        
        if not self.migration_urge:
            self.weights[-1] = 0
        
        # Lista brzina mora pratiti listu težina
        velocities = []
        velocities.append(self.object_dodge_vel(2, neighborhood_angle) )
        velocities.append(self.separation_vel(neighborhood_distance, neighborhood_angle, sep_exp = sep_exp) )
        velocities.append(self.alignment_vel(neighborhood_distance, neighborhood_angle) )
        velocities.append(self.cohesion_vel(neighborhood_distance, neighborhood_angle) )
        mig_vel, mig_dist = self.migration_goal_vel()
        velocities.append(mig_vel)

        desired_velocity = np.array([0.0, 0.0])
        if mode == "priority":
            for vel in velocities:
                if np.linalg.norm(desired_velocity + vel) < self.max_vel:
                    desired_velocity += vel
                else:
                    diff = self.max_vel - np.linalg.norm(desired_velocity)
                    desired_velocity += vel/(np.linalg.norm(vel) + 0.000001) * diff
                    
                    break
        elif mode == "weighted": 
            for vel, weight in zip(velocities, self.weights): #
                desired_velocity += vel * weight
        
        twist = Twist()
        desired_velocity += self.velocity 
        
        if np.linalg.norm(desired_velocity) > self.max_vel: # Clippanje brzine prema potrebama zadatka/robota
            desired_velocity = desired_velocity / np.linalg.norm(desired_velocity)  * self.max_vel
            
        if self.migration_urge and mig_dist < 1: # Arrival s tahn gušenjem
            desired_velocity = desired_velocity * (np.tanh(0.5* (np.max(mig_dist - 0.05, 0)) ))
            
      
        twist.linear.x = desired_velocity[0]
        twist.linear.y = desired_velocity[1]
        twist.angular.z = 0.0  
        self.cmd_vel_pub.publish(twist)
        self.velocity = desired_velocity
        
        

def main():
    rospy.init_node("reynolds_controller", anonymous=True)

    # Kontrolni parametri
    num_robots = 10
    # 0.8 za otvorene mape, 0.5-0.6 za skučene
    neighborhood_distance = 0.8
    # distance**sep_exp određuje čemu je separacijska sila obrnuto proporcionalna, 1 bez migration_urge, 1.4-1.6 inače
    sep_exp = 1.6
    # FoV
    neighborhood_angle = 320
    # Maksimalna brzina, 3 za otvorene mape, 1 za skučene
    max_vel = 1

    robots = [ReynoldsRules(robot_id=i, num_robots=num_robots, migration_urge = True, max_vel = max_vel) for i in range(num_robots)]
    while not rospy.is_shutdown():
        for robot in robots:
            robot.compute_vel(neighborhood_distance, neighborhood_angle, sep_exp = sep_exp)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
