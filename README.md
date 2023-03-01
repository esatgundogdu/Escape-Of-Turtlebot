# Escape-Of-Turtlebot
This repository includes the solution for the "Escape of Turtlebot" problem which is an assignment given in Robotics course at Istanbul Technical University. Details about the assignment is given in file named "escape_of_turtlebot.pdf". The solution of the problem is in "assignment1/a1_answer.py".

## Solution report
I used the second method as I thought it would finish the maze faster. The first
method would be easier to implement, but would take much longer for the robot to
finish the maze. That's why I chose the second method. First, I determined the states
that the robot might encounter in the maze. I divided the robot's movements into
modes. The robot makes its movement according to the current mode. It changes
mode in some states it encounters.
1. FACE_GOAL: The algorithm starts from this mode. In this mode, the robot
first calculates the required direction for the robot to reach the target from its
current position. Then it determines that it should turn right or left to reach that
direction by the shortest path. Finally, the robot turns towards the target.
When it arrives at the destination, it switches to MOVE_FORWARD mode.
2. MOVE_FORWARD: In this mode, the robot moves in the direction it is until
there is an obstacle in front of it. When it encounters a wall, it calculates the
slope of the wall with the getWallYaw() function before proceeding to the next
step. It then switches to PARALLELING_WALL mode to be parallel to the
wall.
3. PARALLLELING_WALL: In this mod, the robot calculates the difference of
robot’s yaw and goal yaw in every loop iteration. Giving the yaw difference to
isFacingGoal() returns if the robot has reached wall yaw. If not, the robot
rotates around itself. If it reaches wall yaw, it changes its mode to
MOVE_PARALLEL.
4. MOVE_PARALLEL: In this mode, the robot moves parallel to the wall until it
reaches the leave point. It checks if its position is on “initial position - goal line”
in every loop iteration.
a. If it reaches the leave point, it will change mode to FACE_GOAL and
restart all the process (This repeats until the robot reaches the goal).
b. If the robot hits a wall during parallel moving, it will save the obstacles
yaw and change mode to PARALLELING_WALL.
c. If the robot reaches the turning point (corner of the obstacle) it changes
its mode to TURNING_CORNER mode.
5. TURNING_CORNER: The robot moves slightly away from the wall, rotates 90
degrees and changes mode to MOVE_CORNER.
6. MOVE_CORNER: The robot moves forward until it reaches the wall level.
After that changes mode to MOVE_PARALLEL.

### Helping functions:
1. getWallYaw(ranges, wall_point_thld): Starts from ranges[0] and
ranges[1]. Iterates over positive side neighbor points and checks if the
point is on the line that passes from previous points. Uses
linearRegression method to find the slope of the line and returns the
yaw value.
2. linearRegression(xlist, ylisy): Basic linear regression method, returns
the intercept, slope and error of the calculated line.
3. isPointOnLine(pointX, pointY, x1, y1, x2, y2, threshold): Checks the
distance between the point and the line that passes through (x1, y1)
and (x2, y2) points. If the distance under threshold, returns true.
4. calculateCoords(range, angle): Returns 2D coordinates from given
LaserScan range and angle values.
5. isReachedGoal(goal): Checks the distance between the goal and
position of the robot.
6. isFacingGoal(diffYaw): Returns the robot facing the goal.
7. rotate_to_goal(diffYaw, maxVel): Determines the direction and
magnitude of turning velocity.
8. get_yaw_to_goal(goal): Returns the yaw of goal.
