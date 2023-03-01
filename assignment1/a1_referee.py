from re import X
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
from time import time

class Referee(Node):

    def __init__(self):
        super().__init__('a1_referee')

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal = {'x': -1,'y':-2}
        self.move_counter = 0
        self.goal_achieved_counter = 0
        self.goal_dist_thld = 0.2 # max acceptable distance between robot and the goal
        self.start = -1
        self.end = -1
        self.hasStarted = False
    def map_callback(self, msg):
        pass


    def odom_callback(self, msg):
        global robotX
        global robotY
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
    
        velocity_vec = Twist() # to fill the velocity message 

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        robotX_tf = t.transform.translation.x
        robotY_tf = t.transform.translation.y

        self.dist_to_goal= round(
            math.sqrt((
            ((self.goal['x']- robotX_tf)**2) + (
            (self.goal['y'] - robotY_tf)**2))),2) 
        self.get_logger().info('Distance to goal:'+ str(self.dist_to_goal),throttle_duration_sec=1)
        if (self.hasStarted):
            self.get_logger().info('Elapsed secs:'+ str(time() - self.start),throttle_duration_sec=1)
        if (self.dist_to_goal <= self.goal_dist_thld):
            self.get_logger().info('GOAL IS ACHIEVED! STOP TURTLEBOT!')
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = 0.0
            self.publish_twist.publish(velocity_vec)
            if (self.goal_achieved_counter==0):
                self.end = time()
                elapsed_time = self.end - self.start
                with open('time.txt', 'w') as f:
                    f.write(str(elapsed_time))
                f.close()
            self.goal_achieved_counter+=1
        if ((round(msg.twist.twist.linear.x,2) > 0.0) | (round(msg.twist.twist.angular.z,2) > 0.0)):
            
            if (self.move_counter==0):
                self.get_logger().info('Clock has started, RUN TURTLEBOT')
                self.start = time()
                self.hasStarted = True
            self.move_counter+=1

def main(args=None):
    rclpy.init(args=args)

    ref = Referee()

    rclpy.spin(ref)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    
    ref.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()