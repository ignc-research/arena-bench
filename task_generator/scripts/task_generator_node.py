#! /usr/bin/env python3

import rospkg
import rospy
import time
from nav_msgs.msg import Odometry
from task_generator.tasks import get_predefined_task
from std_msgs.msg import Int16, Bool
from nav_msgs.srv import LoadMap
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, SpawnModelRequest, SpawnModel, DeleteModel
from task_generator.utils import *
import subprocess

# for clearing costmap
from clear_costmap import clear_costmaps
import pathlib
import re


class TaskGenerator:
    def __init__(self):

        self.sr = rospy.Publisher("/scenario_reset", Int16, queue_size=1)
        self.nr = 0
        mode = rospy.get_param("~task_mode")
        
        scenarios_json_path = rospy.get_param("~scenarios_json_path")
        paths = {"scenario": scenarios_json_path}
        self.task = get_predefined_task("", mode, PATHS=paths)
        self.rospack = rospkg.RosPack()

        # if auto_reset is set to true, the task generator will automatically reset the task
        # this can be activated only when the mode set to 'ScenarioTask'
        auto_reset = rospy.get_param("~auto_reset")
        self.start_time_ = rospy.get_time()

        self.pub = rospy.Publisher('End_of_scenario', Bool, queue_size=10)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.first_round = True
        # if the distance between the robot and goal_pos is smaller than this value, task will be reset
        self.timeout_ = rospy.get_param("~timeout", 2.0) * 60  # sec
        self.delta_ = rospy.get_param("~delta")
        robot_odom_topic_name = rospy.get_param("robot_odom_topic_name", "odom")

        auto_reset = auto_reset and (mode == "scenario" or mode == "random")
        self.curr_goal_pos_ = None

        if auto_reset:
            rospy.loginfo(
                "Task Generator is set to auto_reset mode, Task will be automatically reset as the robot approaching the goal_pos"
            )
            self.reset_task()
            self.robot_pos_sub_ = rospy.Subscriber(
                robot_odom_topic_name, Odometry, self.check_robot_pos_callback
            )

            rospy.Timer(rospy.Duration(0.5), self.goal_reached)

        else:
            # declare new service task_generator, request are handled in callback task generate
            self.reset_task()
            self.task_generator_srv_ = rospy.Service(
                "task_generator", Empty, self.reset_srv_callback
            )

        self.err_g = 100

    def goal_reached(self, event):

        if self.err_g < self.delta_:
            print(self.err_g)
            print("reached goal")
            self.reset_task()
        if rospy.get_time() - self.start_time_ > self.timeout_:
            print("timeout")
            self.reset_task()

    def clear_costmaps(self):
        bashCommand = "rosservice call /move_base/clear_costmaps"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print([output, error])

    def reset_srv_callback(self, req):
        rospy.loginfo("Task Generator received task-reset request!")
        self.task.reset()
        # return EmptyResponse()

    def reset_task(self):

        self.start_time_ = rospy.get_time()
        if not self.first_round: self.pause()
        info = self.task.reset()
        if not  self.first_round: self.unpause()
        # set goal position
        self.sr.publish(self.nr)
        if info is not None:
            if info == "End":
                # communicates to launch_arena (if used) the end of the simulation
                print("SENDING END MESSAGE")
                self.end_msg = Bool()
                self.end_msg.data = True
                self.pub.publish(self.end_msg)
                rospy.signal_shutdown("Finished all episodes of the current scenario")
            else:
                self.curr_goal_pos_ = info["robot_goal_pos"]
        rospy.loginfo("".join(["="] * 80))
        rospy.loginfo("goal reached and task reset!")
        rospy.loginfo("".join(["="] * 80))
        self.nr += 1

    def check_robot_pos_callback(self, odom_msg):
        # type: (Odometry) -> Any
        robot_pos = odom_msg.pose.pose.position
        robot_x = robot_pos.x
        robot_y = robot_pos.y
        goal_x = self.curr_goal_pos_[0]
        goal_y = self.curr_goal_pos_[1]

        self.err_g = (robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2


if __name__ == "__main__":
    rospy.init_node("task_generator")
    rospy.wait_for_service("/static_map")
    task_generator = TaskGenerator()
    rospy.spin()
