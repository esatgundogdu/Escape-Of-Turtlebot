## EB
## a1_answer.py
## 
## BLG456E Assignment 1 Skeleton
##
## Instructions: Change the scan_callback and odom_callback function to make the robot navigate to the goal
## using desired motion planning algorithm (Bug1 or Bug2). You may add helper functions to make code seem more clean, it would also
## help me to easily examine :))
## 
## Notes to consier: Few helper functions and code sniipets are already given to you. Examine the code carefully beforehand.
##
## Extra: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## 
## STUDENT_ID:<150190055>
from socket import IP_TRANSPARENT
import rclpy
from rclpy.node import Node
import sys
from rosidl_runtime_py import import_message_from_namespaced_type
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray

"""
HELPER FUNCTIONS
"""
class MoveMode(Enum):
    FACE_GOAL = 0
    MOVE_FORWARD = 1
    PARALLELING_WALL = 2
    MOVE_PARALLEL = 3
    TURNING_CORNER = 4
    MOVE_CORNER = 5

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

# returns yaw value of the goal
def get_yaw_to_goal(goal):
    difX = goal.get("x") - robotX_tf
    difY = goal.get("y") - robotY_tf
    
    yaw = math.atan2(difY, difX)
    return yaw

# returns angular velocity for safe turning (turns slower on close differences)
def rotate_to_goal(diffYaw, maxVel = 0.9):
    angular_vel = 0.0
    if abs(diffYaw) > 2*math.pi - 0.25:
        angular_vel = min(0.1, maxVel)
    elif abs(diffYaw) > 0.25:
       angular_vel = maxVel
    else:
        angular_vel = min(0.1, maxVel)

    # turn right if its closer 
    if 0 > diffYaw > -math.pi or math.pi < diffYaw < 2*math.pi:
        angular_vel *= -1
    
    return angular_vel
 
# check if given yaw difference is enough by threshold
def isFacingGoal(diffYaw):
    threshold = 0.01
    if abs(diffYaw) < threshold or abs(diffYaw) > 2*math.pi - threshold:
        return True
    else:
        return False
 
# check if robot has reached goal
def isReachedGoal(goal):
    distX = goal.get("x") - robotX_tf
    distY = goal.get("y") - robotY_tf
    dist = math.sqrt(distX ** 2 + distY ** 2)

    if dist <= 0.2:
        return True

    return False

# calculates x and y coordinates from laserscan angle and range
def calculateCoords(dist, angle):
    angle_yaw = angle / 180 * math.pi
    x = dist * math.cos(robot_yaw + angle_yaw) + robotX_tf
    y = dist * math.sin(robot_yaw + angle_yaw) + robotY_tf
    return (x, y)

# returns distance between two points
def getDistance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

# returns slope, intercept and error of the regression line, used for calculating the wall angle
def linearRegression(x, y):
    # number of points
    n = len(x)

    # mean of x and y vector
    x_mean = sum(x) / n
    y_mean = sum(y) / n

    # calculating cross-deviation and deviation about x
    SS_xy = 0
    SS_xx = 0
    for i in range(n):
        SS_xy += (x[i] - x_mean) * (y[i] - y_mean)
        SS_xx += (x[i] - x_mean) * (x[i] - x_mean)

    # calculating regression coefficients
    b_1 = SS_xy / SS_xx
    b_0 = y_mean - b_1 * x_mean

    # if line is vertical maybe too small

    # calculate error of each point
    # if error is too high, then the point is an outlier
    error = []
    for i in range(n):
        error.append(abs(y[i] - (b_0 + b_1 * x[i])))

    stDevX = math.sqrt(SS_xx / n)
    stDevY = math.sqrt(sum([e**2 for e in error]) / n)

    verticality = stDevY / stDevX

    isVertical = verticality > 1.5

    # if line is vertical, get slope of inverse line
    if isVertical:
        intercept, slope, e = linearRegression(y, x)
        if slope < 0:
            slope = -slope
        return (-intercept / slope, 1 / slope, e)

    # equation: y = b_0 + b_1 * x
    # slope: b_1
    # intercept: b_0
    return (b_0, b_1, error)

# returns the yaw value of the obstacle
def getWallYaw(ranges, wall_point_thld, pub_marker):
    x0, y0 = calculateCoords(ranges[0], 0)
    xlist, ylist = [x0], [y0]
    n = 90
    # For positive side (0 to n)
    x2, y2 = calculateCoords(ranges[1], 1)
    if getDistance(x0, y0, x2, y2) < wall_point_thld:
        xlist.append(x2)
        ylist.append(y2)
        for i in range(1, n):
            if ranges[i] == float('Inf'):
                break
            x_coord, y_coord = calculateCoords(ranges[i], i)
            dist = getDistance(x_coord, y_coord, xlist[-1], ylist[-1])
            if dist < wall_point_thld:
                # check if x difference and y difference are same
                # if not, then it is not a wall
                if isPointOnLine(x_coord, y_coord, xlist[-1], ylist[-1], x0, y0):
                    xlist.append(x_coord)
                    ylist.append(y_coord)
            else:
                break

    # if not enough points, then return None
    if len(xlist) < 3:
        return None

    intercept, slope, error = linearRegression(xlist, ylist)

    publishMarkers(xlist, ylist, pub_marker)

    
    wallYaw = math.atan(slope) 
    return wallYaw

# For debugging, publishes given x and y lists as markers
def publishMarkers(xlist, ylist, pub_marker):
    pub_marker.publish(MarkerArray())
    markerArray = MarkerArray()
    for i in range(len(xlist)):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = xlist[i]
        marker.pose.position.y = ylist[i]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0 / (i + 1) / 0.015
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.id = i
        markerArray.markers.append(marker)
    pub_marker.publish(markerArray)

# calculate distance between given point and line
def isPointOnLine(pointX, pointY, x1, y1, x2, y2, threshold = 0.1):
    dist = abs((y2-y1)*pointX - (x2-x1)*pointY + x2*y1 - y2*x1) / math.sqrt((y2-y1)**2 + (x2-x1)**2)
    if dist < threshold:
        return True
    else:
        return False


robot_yaw, robotX, robotY, robotX_tf, robotY_tf = 0, 0, 0, 0, 0
wall_yaw = 0
turning_point = (0, 0)
initialPoseFlag = True

class Navigator(Node):
    """
    Navigator node to make robot go from location A to B. 
    [IMPORTANT]
    IMPLEMENT YOUR CODES WITHIN THIS CLASS (You can define helper functions outside the class if you want)
    [IMPORTANT]
    """
    def __init__(self):
        super().__init__('a1_answer')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
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
        self.pub_marker = self.create_publisher(MarkerArray, 'debug_markers', 10)

        self.tf_buffer = Buffer() # for transformation
        self.tf_listener = TransformListener(self.tf_buffer, self) # for transformation
        # DO NOT CHANGE self.goal!
        self.goal = {'x': -1,'y':-2} # DO NOT CHANGE! REFEREE EXECUTABLE WILL CHECK THIS GOAL!
        # DO NOT CHANGE self.goal!
        self.goal_dist_thld = 0.2 # max acceptable distance between robot and the goal
        self.min_collision_dist = 0.8
        self.wall_point_thld = 0.15
        self.linear_vel = 0.6
        self.parallel_vel = 0.7
        self.moveMode = MoveMode.FACE_GOAL
        self.facing = False
        self.reached = False
        self.isParallel = False
        self.hitPointPassed = False
        self.initialPoseX = 0
        self.initialPoseY = 0
        self.hitPoint = (0, 0)


    def scan_callback(self, msg):
        velocity_vec = Twist()

        global wall_yaw, turning_point

        # check if robot reached the goal 
        self.reached = isReachedGoal(self.goal)
        if not self.reached: # if not reached goal, run algorithm

            # robot rotates to face goal, after that, it will go to MOVE_FORWARD state.
            if self.moveMode == MoveMode.FACE_GOAL: 
                # get the yaw value that robot should face to reach the goal
                goal_yaw = get_yaw_to_goal(self.goal)
                diffYaw = goal_yaw - robot_yaw
                
                # check if robot is facing the goal
                self.facing = isFacingGoal(diffYaw)
                
                if not self.facing: # rotate to face goal
                    velocity_vec.angular.z = rotate_to_goal(diffYaw)
                else: # change mode to next
                    self.get_logger().info("Changing mode to MOVE_FORWARD")
                    self.moveMode = MoveMode.MOVE_FORWARD
            
            # robot moves forward until it hits a wall, after this state, it will go to PARALLELING_WALL state.
            elif self.moveMode == MoveMode.MOVE_FORWARD: 
                # check if robot is close to a wall 
                if msg.ranges[0] > self.min_collision_dist:
                    velocity_vec.linear.x = self.linear_vel
                else:
                    velocity_vec.linear.x = 0.0
                    # get the yaw value that robot should face to be parallel to the wall
                    wall_yaw = getWallYaw(msg.ranges, self.wall_point_thld, self.pub_marker)
                    while wall_yaw == None:
                        wall_yaw = getWallYaw(msg.ranges, self.wall_point_thld, self.pub_marker)
                    # save the point that robot stops moving forward
                    self.hitPoint = (robotX_tf, robotY_tf)
                    self.hitPointPassed = False

                    self.get_logger().info("Changing mode to PARALLELING_WALL")
                    self.moveMode = MoveMode.PARALLELING_WALL
            
            # robot rotates until it is parallel to the wall, after this state, it will go to MOVE_PARALLEL state.
            elif self.moveMode == MoveMode.PARALLELING_WALL:
                diffYaw = wall_yaw - robot_yaw

                # check if robot is parallel to the wall
                self.isParallel = isFacingGoal(diffYaw)
                if not self.isParallel:
                    # if robot is not parallel, rotate with necessary velocity to be parallel
                    velocity_vec.angular.z = rotate_to_goal(diffYaw)
                else:
                    # self.get_logger().info("Parallel to wall, wall_yaw: {:.2f}, robot_yaw: {:.2f}".format(wall_yaw, robot_yaw))
                    self.get_logger().info("Changing mode to MOVE_PARALLEL")
                    self.moveMode = MoveMode.MOVE_PARALLEL
            
            # robots moves through the wall until it reaches the turning point or the leave point. 
            # If it reaches the turning point, it will go to TURNING_CORNER state.
            # If it reaches the leave point, it will go to FACE_GOAL state.
            # If it hits a wall during following the obstacle, it will go to PARALLELING_WALL state.
            elif self.moveMode == MoveMode.MOVE_PARALLEL:
                if msg.ranges[-90] > self.min_collision_dist + 0.4:
                    velocity_vec.linear.x = self.linear_vel
                    wall_yaw = wall_yaw - math.pi / 2

                    if wall_yaw < -math.pi:
                        wall_yaw += 2 * math.pi

                    turning_point = (robotX_tf, robotY_tf)
                    self.get_logger().info("Changing mode to TURNING_CORNER")
                    self.moveMode = MoveMode.TURNING_CORNER

                # check if robot encountered an obstacle during parallel movement
                elif msg.ranges[0] < self.min_collision_dist:
                    velocity_vec.linear.x = 0.0

                    # get the yaw value that robot should face to be parallel to the wall
                    wall_yaw = getWallYaw(msg.ranges, self.wall_point_thld, self.pub_marker)
                    while wall_yaw == None:
                        wall_yaw = getWallYaw(msg.ranges, self.wall_point_thld, self.pub_marker)

                    # get opposite wall yaw
                    if wall_yaw < math.pi / 2 - 0.05:
                        wall_yaw = wall_yaw + math.pi

                    if abs(wall_yaw - math.pi / 2) < 0.05:
                        # if robots direction is left, wall_yaw = -pi/2
                        if abs(robot_yaw - math.pi) < 0.05 or abs(robot_yaw + math.pi) < 0.05:
                            wall_yaw = -math.pi / 2

                    self.get_logger().info("Changing mode to PARALLELING_WALL")
                    self.moveMode = MoveMode.PARALLELING_WALL
                else:
                    # if robot is disoriented, rotate to be parallel to wall
                    velocity_vec.angular.z = rotate_to_goal(wall_yaw - robot_yaw, 0.15)
                    if msg.ranges[0] < self.min_collision_dist + 0.4:
                        velocity_vec.linear.x = min(self.parallel_vel / 2, 0.4)
                    else:
                        velocity_vec.linear.x = self.parallel_vel
                
                    # check if robot pose is on the line between initial pose and goal
                    if not self.hitPointPassed:
                        hitPointDist = getDistance(robotX_tf, robotY_tf, self.hitPoint[0], self.hitPoint[1])
                        if hitPointDist > 0.5: # Hit point passed
                            self.hitPointPassed = True
                    else:
                        # check if robot is on leave point, if so, change mode to FACE_GOAL
                        if isPointOnLine(robotX_tf, robotY_tf, self.initialPoseX, self.initialPoseY, self.goal.get("x"), self.goal.get("y"), 0.15):
                            self.get_logger().info("Changing mode to FACE_GOAL")
                            self.moveMode = MoveMode.FACE_GOAL
            
            # robot rotates until it is parallel to the other side of wall, after this state, it will go to MOVE_CORNER state.
            elif self.moveMode == MoveMode.TURNING_CORNER:
                dist = getDistance(robotX, robotY, turning_point[0], turning_point[1])
                
                if dist > 0.4:
                    diffYaw = wall_yaw - robot_yaw

                    facing = isFacingGoal(diffYaw)

                    if not facing:
                        velocity_vec.angular.z = rotate_to_goal(diffYaw)
                    else:
                        self.get_logger().info("Changing mode to MOVE_CORNER")
                        self.moveMode = MoveMode.MOVE_CORNER
                else:
                    velocity_vec.angular.z = 0.0
                    velocity_vec.linear.x = self.linear_vel

            # robot moves until it reaches the wall level. After this state, it will go to MOVE_PARALLEL state.
            elif self.moveMode == MoveMode.MOVE_CORNER:
                if msg.ranges[-90] < self.min_collision_dist + 0.4:
                        velocity_vec.linear.x = 0.0
                        self.get_logger().info("Changing mode to MOVE_PARALLEL")
                        self.moveMode = MoveMode.MOVE_PARALLEL
                else:
                    velocity_vec.linear.x = self.linear_vel

        # If robot reaches the goal, it will stop.
        else:
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = 0.0
            self.get_logger().info("Reached goal")
        self.publish_twist.publish(velocity_vec) # publish twist message through cmd_vel topic


        
    ## You may also make use of the map which is being built by the "turtlebot3_cartographer" 
    ## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
    ## If you want me to explain the data structure, I will - just ask me in advance of class
    def map_callback(self, msg):

        chatty_map = False # change to true if you want to examine the map in a compressed way (shrinked to fit into terminal)
        if chatty_map:
            print ("-------MAP---------")
            ## Here x and y has been incremented with five to make it fit in the terminal
            ## Note that we have lost some map information by shrinking the data
            for x in range(0,msg.info.width-1,5):
                for y in range(0,msg.info.height-1,5):
                    index = x+y*msg.info.width
                    if msg.data[index] > 50:
                        ## This square is occupied
                        sys.stdout.write('X')
                    elif msg.data[index] >= 0:
                        ## This square is unoccupied
                        sys.stdout.write(' ')
                    else:
                        sys.stdout.write('?')
                sys.stdout.write('\n')
            sys.stdout.flush()
            print ("-------------------")
        pass

    def odom_callback(self, msg):
        global robotX # global keyword makes the variable accessable even outside the function!
        global robotY # global keyword makes the variable accessable even outside the function!
        global robotX_tf # global keyword makes the variable accessable even outside the function!
        global robotY_tf # global keyword makes the variable accessable even outside the function!
        global robot_yaw # global keyword makes the variable accessable even outside the function!
        global initialPoseFlag

        
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
        try:
            # grab the latest available transform from the odometry frame 
            # (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        robotX_tf = t.transform.translation.x
        robotY_tf = t.transform.translation.y
        robot_yaw = robot_orient_z # # only need the z axis, degree of orientation, between pi and -pi
        if initialPoseFlag:
            self.initialPoseX = robotX
            self.initialPoseY = robotY
            initialPoseFlag = False
        # self.get_logger().info('X:'+str(robotX_tf),throttle_duration_sec=0.5) # once at a half of a second
        # self.get_logger().info('Y:'+str(robotY_tf),throttle_duration_sec=0.5) # once at a half of a second
        # self.get_logger().info('Yaw:'+str(robot_yaw),throttle_duration_sec=0.5) # once at a half of a second

        

        
def main(args=None):
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    rclpy.init(args=args)

    navigator_node = Navigator()
    rclpy.spin(navigator_node) 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()