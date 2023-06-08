#!/usr/bin/env python3

import math
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

LOOK_AT_ACTION_NAME = '/head_controller/point_head' 
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory'
PAN_JOINT = 'head_pan_joint' 
TILT_JOINT = 'head_tilt_joint' 
PAN_TILT_TIME = 1.0


class Head(object):
    """Head controls the Fetch's head.
    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians
    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)"""

    def __init__(self):
        self.look_client = actionlib.SimpleActionClient(
                LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        self.pan_client = actionlib.SimpleActionClient(
                PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self.look_client.wait_for_server()
        self.pan_client.wait_for_server()
        self.MIN_PAN = -1.570796 
        self.MAX_PAN = 1.570796 
        self.MIN_TILT = -0.7853982 
        self.MAX_TILT = 1.570796 

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.
        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at."""
        goal = control_msgs.msg.PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1)
        self.look_client.send_goal(goal)
        self.look_client.wait_for_result()
        return self.look_client.get_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.
              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        if pan < self.MIN_PAN or pan > self.MAX_PAN:
            return
        if tilt < self.MIN_TILT or tilt > self.MAX_TILT:
            return
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(pan)
        point.positions.append(tilt)
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(PAN_JOINT)
        goal.trajectory.joint_names.append(TILT_JOINT)
        goal.trajectory.points.append(point)
        self.pan_client.cancel_goal() # preempt last goal
        self.pan_client.send_goal(goal)
        # self.pan_client.wait_for_result()
        # return self.pan_client.get_result()

