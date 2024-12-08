#!/usr/bin/env python3
import rospy
from pynput import mouse
from geometry_msgs.msg import Point

# Dependant on individual setup, change accordingly!
TOP_LEFT_X = 716  
TOP_LEFT_Y = 261  
BOTTOM_RIGHT_X = 1133  
BOTTOM_RIGHT_Y = 677  

REGION_WIDTH = BOTTOM_RIGHT_X - TOP_LEFT_X
REGION_HEIGHT = BOTTOM_RIGHT_Y - TOP_LEFT_Y

SIM_WIDTH = 10.  # Simulator width 
SIM_HEIGHT = 10.  # Simulator height 
SIM_ORIGIN_X = 5.  # Simulator origin X-coordinate
SIM_ORIGIN_Y = 5. # Simulator origin Y-coordinate
RESOLUTION = 0.05


def on_move(x, y):
    # Check if the mouse within defined region
    if TOP_LEFT_X <= x <= BOTTOM_RIGHT_X and TOP_LEFT_Y <= y <= BOTTOM_RIGHT_Y:
        # Screen to simulation coordinate transform
        scaled_x = ((x - TOP_LEFT_X) / REGION_WIDTH)* SIM_WIDTH -SIM_ORIGIN_X
        scaled_y = ((REGION_HEIGHT - (y - TOP_LEFT_Y)) / REGION_HEIGHT)* SIM_WIDTH -SIM_ORIGIN_Y
        
        
        rospy.loginfo(f"Mouse moved to ({x}, {y}) -> Scaled position: ({scaled_x}, {scaled_y})")
        pub.publish(Point(scaled_x, scaled_y, 0))

if __name__ == '__main__':
    rospy.init_node('mouse_goal_publisher', anonymous=True)
    pub = rospy.Publisher('migration_goal', Point, queue_size=10)

    with mouse.Listener(on_move=on_move) as listener:
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            listener.stop()
