#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import math

	
class ReynoldsRules:
    def __init__(self, robot_id, num_robots, dodge_weight = 1, separation_weight=1, alignment_weight=0.7, cohesion_weight=0.2):
        self.robot_id = robot_id
        self.num_robots = num_robots

        self.weights = [dodge_weight, separation_weight, alignment_weight, cohesion_weight]


        self.position = np.array([0.0, 0.0])
			
        self.all_positions = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}
        self.all_velocities = {i: np.array([math.cos(i / num_robots * 2 * math.pi), math.sin(i / num_robots * 2 * math.pi)]) for i in range(num_robots)}

		
		
        self.odom_sub = rospy.Subscriber("robot_" +str(robot_id) + "/odom", Odometry, self.odom_callback, callback_args=robot_id)

        for boid_id in range(num_robots):
            if boid_id != robot_id:
                rospy.Subscriber("robot_" + str(boid_id) + "/odom", Odometry, self.odom_callback, callback_args=boid_id)
                rospy.Subscriber(f"robot_{boid_id}/cmd_vel", Twist, self.cmd_vel_callback, callback_args=boid_id)
        
        
        self.cmd_vel_pub = rospy.Publisher(f"robot_{robot_id}/cmd_vel", Twist, queue_size=10)
        
        #vel = np.array([np.random.uniform(-0.5, 0.5),np.random.uniform(-0.5, 0.5)])
        vel = np.array([math.cos(robot_id / num_robots * 2 * math.pi), math.sin(robot_id / num_robots * 2 * math.pi)])
        twist = Twist()
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.angular.z = 0.0  
        self.cmd_vel_pub.publish(twist)
        
        self.velocity = vel
        
        
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
    def get_nearest_obstacle_in_front(self, range_ahead=5.0, num_rays=3):
        # check if map is loaded, default detection range 5 m, default number of rays is 3
        if self.map_data is None:
            return None
        
        # convert the boids's position from world coordinates to map grid coordinates
        map_x = int((self.position[0] - self.map_origin[0]) / self.map_resolution)
        map_y = int((self.position[1] - self.map_origin[1]) / self.map_resolution)
        
        # calculate angle of the boids's velocity vector and generate angles for detection rays
        velocity_angle = np.arctan2(self.velocity[1], self.velocity[0])
        angles = np.linspace(velocity_angle - np.pi / 6, velocity_angle + np.pi / 6, num_rays)  # -30 0 +30
        
        obstacle_distances = [] # empty list to store the distances to obstacles along each ray
        for angle in angles:
            nearest_obstacle_distance = float('inf')  # initialize the distance as infinity (can't see obstacle)

            # trace a ray outward from the boid's position along the current angle
            for i in range(1, int(range_ahead / self.map_resolution)):  
                x_offset = i * math.cos(angle) * self.map_resolution
                y_offset = i * math.sin(angle) * self.map_resolution

                # calculate the grid cell being checked
                ray_x = int(map_x + x_offset / self.map_resolution)
                ray_y = int(map_y + y_offset / self.map_resolution)

                # check if cell is within map bounds
                if 0 <= ray_x < self.map_data.shape[1] and 0 <= ray_y < self.map_data.shape[0]:
                     # if the cell is occupied (value 100), update nearest obstacle distance
                    if self.map_data[ray_y, ray_x] == 100:   
                        nearest_obstacle_distance = min(nearest_obstacle_distance, i * self.map_resolution)
                        break  # break after detection
            # if detected, append the distance, if not, set it to infy
            if nearest_obstacle_distance != float('inf'):
                obstacle_distances.append(nearest_obstacle_distance)
            else:
                obstacle_distances.append(float('inf'))
        return obstacle_distances
        
        
    def object_dodge_vel(self, neighborhood_distance = 2.0, neighborhood_angle = 180):
        obstacle_distances = self.get_nearest_obstacle_in_front()
        
        dodge_force = np.array([0.0, 0.0]) # initialize the dodge force vector as zero [x y]

        # check if the middle ray detects an obstacle within the neighborhood distance
        if obstacle_distances[1] < neighborhood_distance:
            # then compare this with other rays to see which side is closest to the obstacle
            if obstacle_distances[0] > obstacle_distances[2]:
                # if the left is clearer, steer right
                dodge_force += np.matmul(np.array([[0, 1], [-1, 0]]), self.velocity) # rotate velocity clockwise
            else: 
                # if the right is clearer, steer left
                dodge_force += np.matmul(np.array([[0, -1], [1, 0]]), self.velocity) # rotate counter-clockwise

            # subtract the current velocity to focus only on the dodge adjustment
            dodge_force -= self.velocity

            # scaling this velocity inversly with the cube of the distance (the closer the boid is, the stronger the force)
            dodge_force = dodge_force / obstacle_distances[0]**3
        
            
        return dodge_force
        
    def odom_callback(self, msg, robot_id):
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


    def alignment_vel(self, neighborhood_distance=1.0, neighborhood_angle = 180):
        average_velocity = np.array([0.0, 0.0])
        count = 0
        for i, (pos, vel) in enumerate(zip(self.all_positions.values(), self.all_velocities.values())):
            if i != self.robot_id:  
                distance = np.linalg.norm(self.position - pos)

                
                if distance < neighborhood_distance :
                    average_velocity += vel
                    count += 1
        if count > 0:
            average_velocity /= count
            return average_velocity - self.velocity  
        else: #ako nema susjeda, nema ni aligmenta
            return np.array([0.0, 0.0])


    def cohesion_vel(self, neighborhood_distance=1.0, neighborhood_angle = 180):
        center_of_mass = np.array([0.0, 0.0])
        count = 0
        for i, pos in self.all_positions.items():
            if i != self.robot_id:  
                distance = np.linalg.norm(self.position - pos)

                
                if distance < neighborhood_distance :
                    center_of_mass += pos
                    count += 1
        if count > 0:
            center_of_mass /= count
            return center_of_mass - self.position  
        else: #opet, ako nema susjeda, nema kohezije
            return np.array([0.0, 0.0])
        

    def separation_vel(self, neighborhood_distance=1.0, neighborhood_angle = 180):
        separation_force = np.array([0.0, 0.0])
        for i, pos in self.all_positions.items():
            if i != self.robot_id: 
                distance = np.linalg.norm(self.position - pos) + 0.0001 #da ne cmizdri za div by zero

                if distance < neighborhood_distance / 2 :
                    separation_force += (self.position - pos) / (distance)**2 
        return separation_force


    def compute_vel(self, neighborhood_distance=1.0, neighborhood_angle = 180):
        mode = "priority"
        max_speed = 1
        
        velocities = []
        velocities.append(self.object_dodge_vel(2, neighborhood_angle) )
        velocities.append(self.separation_vel(neighborhood_distance, neighborhood_angle) )
        velocities.append(self.alignment_vel(neighborhood_distance, neighborhood_angle) )
        velocities.append(self.cohesion_vel(neighborhood_distance, neighborhood_angle) )

        desired_velocity = np.array([0.0, 0.0])
        if mode == "priority":
            for vel in velocities:
                if np.linalg.norm(desired_velocity + vel) < max_speed:
                    desired_velocity += vel
                    if self.robot_id == 1: print("heww")
                else:
                    diff = max_speed - np.linalg.norm(desired_velocity)
                    desired_velocity += vel/(np.linalg.norm(vel) + 0.000001) * diff
                    if self.robot_id == 1: print(desired_velocity)
                    break
        elif mode == "weighted": #ovo radi puno odvratnije, trebao bi se tweakat tezine
            for vel, weight in zip(velocities, self.weights): #podrazumijeva da su u weights tezine isto poredane kako se i racunaju brzine
                desired_velocity += vel * weight
        

        if np.linalg.norm(desired_velocity) < 0.08 and False: #ako oce neku vrlo malo korekciju, nek miruje radije 
            desired_velocity = np.array([0.0, 0.0]) #ovo možda izbacit, meni je služilo za debug
        #elif np.linalg.norm(desired_velocity) > 1: #ovo ne treba kad je priority jer priority automatski clippa na neku velicinu 
            #desired_velocity = desired_velocity / np.linalg.norm(desired_velocity)  
		

        twist = Twist()
        if self.robot_id == 1: print(velocities)
        desired_velocity += self.velocity #ovo je tu samo da ne stanu kad se razmaknu, maknut ovo na kraju, meni je bilo za testiranje
        twist.linear.x = desired_velocity[0]
        twist.linear.y = desired_velocity[1]
        if self.robot_id == 1: 
            print(desired_velocity, "\n")
            print(self.get_nearest_obstacle_in_front(), "aouwdwou")
        
        twist.angular.z = 0.0  
        self.cmd_vel_pub.publish(twist)
        
        self.velocity = desired_velocity
        
        
        
    


def main():
    rospy.init_node("reynolds_controller", anonymous=True)

    num_robots = 10
    neighborhood_distance = 1
    neighborhood_angle = 180

    robots = [ReynoldsRules(robot_id=i, num_robots=num_robots) for i in range(num_robots)]
    while not rospy.is_shutdown():
        for robot in robots:
            robot.compute_vel(neighborhood_distance, neighborhood_angle)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
