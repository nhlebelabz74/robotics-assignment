#!/usr/bin/python

# Mixo Khoza
# Palesa Rapolaki
# Banzile Nhlebela
# Daniel Ngobe

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import sys
import math

from path_planning import rrt_path_planner

# PID gains
Kp_linear = 1.0
Ki_linear = 0.0
Kd_linear = 0.1

Kp_angular = 2.0
Ki_angular = 0.0
Kd_angular = 0.1

class TurtleBotController:
    def __init__(self, target_x, target_y, distance_threshold=0.3):
        self.target_x = target_x
        self.target_y = target_y
        self.distance_threshold = distance_threshold  # Distance threshold for reaching target

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

        self.sum_error_linear = 0.0
        self.sum_error_angular = 0.0

        self.pub_cmd = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        
        # State machine for turn-then-move behavior
        self.TURNING = 0
        self.MOVING = 1
        self.state = self.TURNING

    def state_callback(self, msg):
        try:
            index = msg.name.index("mobile_base")  # Adjust to your model name
            pose = msg.pose[index]
            self.current_x = pose.position.x
            self.current_y = pose.position.y

            # Orientation (quaternion to yaw)
            q = pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_theta = math.atan2(siny_cosp, cosy_cosp)

        except ValueError:
            rospy.logwarn("TurtleBot model not found in Gazebo!")

    def compute_pid(self, target, current, prev_error, sum_error, Kp, Ki, Kd):
        error = target - current
        sum_error += error
        d_error = error - prev_error
        control = Kp * error + Ki * sum_error + Kd * d_error
        return control, error, sum_error

    def move_to_target(self):
        rospy.loginfo("Moving to target: x={:.2f}, y={:.2f}".format(self.target_x, self.target_y))
        
        # Reset state machine and errors
        self.state = self.TURNING
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.sum_error_linear = 0.0
        self.sum_error_angular = 0.0

        while not rospy.is_shutdown():
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)

            # Check if we've reached the target
            if distance < 0.1:
                rospy.loginfo("Target reached.")
                break

            target_theta = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_theta - self.current_theta)

            cmd = Twist()

            if self.state == self.TURNING:
                # TURNING STATE: Only rotate, no forward movement
                rospy.loginfo("Turning to target angle. Angle error: {:.3f} rad ({:.1f} deg)".format(
                    angle_error, math.degrees(angle_error)))
                
                angular_vel, self.prev_error_angular, self.sum_error_angular = self.compute_pid(
                    angle_error, 0, self.prev_error_angular, self.sum_error_angular,
                    Kp_angular, Ki_angular, Kd_angular)
                
                cmd.linear.x = 0.0
                cmd.angular.z = angular_vel
                
                # Switch to moving state when aligned (smaller threshold for better alignment)
                if abs(angle_error) < 0.05:  # ~3 degrees
                    rospy.loginfo("Aligned with target. Starting forward movement.")
                    self.state = self.MOVING
                    # Reset linear PID errors when switching to moving state
                    self.prev_error_linear = 0.0
                    self.sum_error_linear = 0.0

            elif self.state == self.MOVING:
                # MOVING STATE: Move forward with minimal angular correction
                rospy.loginfo("Moving forward. Distance: {:.3f}m".format(distance))
                
                linear_vel, self.prev_error_linear, self.sum_error_linear = self.compute_pid(
                    distance, 0, self.prev_error_linear, self.sum_error_linear,
                    Kp_linear, Ki_linear, Kd_linear)
                
                # Small angular correction to maintain heading
                angular_vel = 0.0
                if abs(angle_error) > 0.02:  # Only correct if significantly off course
                    angular_vel = Kp_angular * 0.3 * angle_error  # Reduced angular gain during movement
                
                cmd.linear.x = max(0.0, min(linear_vel, 0.5))  # Limit maximum speed
                cmd.angular.z = angular_vel
                
                # Switch back to turning if we're too far off course
                if abs(angle_error) > 0.3:  # ~17 degrees
                    rospy.loginfo("Off course. Switching back to turning mode.")
                    self.state = self.TURNING
                    # Reset angular PID errors when switching to turning state
                    self.prev_error_angular = 0.0
                    self.sum_error_angular = 0.0

            self.pub_cmd.publish(cmd)
            self.rate.sleep()

        self.stop()

    def stop(self):
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd.publish(Twist())  # Zero velocities

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def execute(self):
        rospy.sleep(1.0)  # Allow time for setup
        self.move_to_target()

if __name__ == '__main__':
    try:
        rospy.init_node("turtlebot_pid_controller", anonymous=True)

        if len(sys.argv) != 3:
            rospy.logerr("Usage: rosrun <package_name> <script_name> target_x target_y")
            sys.exit(1)

        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])

        path = rrt_path_planner((target_x, target_y))
        if not path:
            rospy.logerr("No path found to the target.")
            sys.exit(1)

        rospy.loginfo("Path found, executing TurtleBot controller...")
        for point in path:
            rospy.loginfo("Path point: x={:.2f}, y={:.2f}".format(point[0], point[1]))
            x, y = point
            # Use 0.3 units as the distance threshold for reaching targets near walls
            controller = TurtleBotController(x, y, distance_threshold=0.5)
            controller.execute()
        
    except rospy.ROSInterruptException:
        pass