#!/usr/bin/env python


import six
import abc
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from threading import Lock
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from tf.transformations import quaternion_from_euler
from .robot_manager import RobotManager
from .obstacle_manager import ObstaclesManager
from .pedsim_manager import PedsimManager
from .ped_manager.ArenaScenario import *
from geometry_msgs.msg import *
from threading import Condition, Lock
from std_srvs.srv import Empty

STANDART_ORIENTATION = quaternion_from_euler(0.0, 0.0, 0.0)
ROBOT_RADIUS = rospy.get_param("radius")

class StopReset(Exception):
    """Raised when The Task can not be reset anymore"""


@six.add_metaclass(abc.ABCMeta)
class ABSTask(abc.ABCMeta("ABC", (object,), {"__slots__": ()})):
    """An abstract class, all tasks must implement reset function."""

    def __init__(self, pedsim_manager, obstacle_manager, robot_manager):
        # type: (PedsimManager, ObstaclesManager, RobotManager) -> None
        self.robot_manager = robot_manager
        self.obstacle_manager = obstacle_manager
        self.pedsim_manager = pedsim_manager
        self._service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
        self._map_lock = Lock()
        rospy.Subscriber("/map", OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abc.abstractmethod
    def reset(self):
        """
        a function to reset the task. Make sure that _map_lock is used.
        """

    def _update_map(self, map_):
        # type (OccupancyGrid) -> None
        with self._map_lock:
            self.obstacle_manager.update_map(map_)
            self.robot_manager.update_map(map_)

class ManualTask(ABSTask):
    """randomly spawn obstacles and user can manually set the goal postion of the robot"""

    def __init__(self, pedsim_manager, obstacle_manager, robot_manager):
        # type: (ObstaclesManager, RobotManager, list) -> None
        super(ManualTask, self).__init__(
            pedsim_manager, obstacle_manager, robot_manager
        )
        # subscribe
        rospy.Subscriber("/manual_goal", PoseStamped, self._set_goal_callback)
        self._goal = PoseStamped()
        self._new_goal_received = False
        self._manual_goal_con = Condition()
        self.num_of_actors = rospy.get_param("~actors", 3)

    def is_True(self):
        if self._new_goal_received is True:
            return True
        else:
            return False

    def reset(self):
        while True:
            with self._map_lock:
                start_pos = self.robot_manager.set_start_pos_random()
                self.obstacle_manager.register_random_dynamic_obstacles(
                    self.num_of_actors,
                    forbidden_zones=[
                        (
                            start_pos.position.x,
                            start_pos.position.y,
                            ROBOT_RADIUS,
                        ),
                    ],
                )
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(
                        lambda: self.is_True() is True, timeout=60
                    )
                    if not self._new_goal_received:
                        raise Exception("TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager._goal_pub.publish(self._goal)
                        self.robot_manager.pub_mvb_goal.publish(self._goal)

                    except Exception as e:
                        rospy.logwarn(repr(e))

    def _set_goal_callback(self, goal: PoseStamped):
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
            self._manual_goal_con.notify()

class ScenarioTask(ABSTask):
    def __init__(self, pedsim_manager, obstacle_manager, robot_manager, scenario_path):
        # type: (PedsimManager, ObstaclesManager, RobotManager, str) -> None
        super(ScenarioTask, self).__init__(
            pedsim_manager, obstacle_manager, robot_manager
        )
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # load scenario from file
        self.scenario = ArenaScenario()
        self.scenario.loadFromFile(scenario_path)

        # setup pedsim agents
        self.pedsim_manager = None
        # if len(self.scenario.pedsimAgents) > 0:
        #     self.pedsim_manager = pedsim_manager
        #     peds = [agent.getPedMsg() for agent in self.scenario.pedsimAgents]
        #     self.pedsim_manager.spawnPeds(peds)
        self.reset_count = 0
        self.unpause()

    def reset(self):
        if self.scenario.resets >= self.reset_count:
            self.reset_count += 1
            info = {}
            with self._map_lock:
                # reset pedsim agents
                # if self.pedsim_manager != None:
                #     self.pedsim_manager.resetAllPeds()

                # reset robot
                self.robot_manager.set_start_pos_goal_pos(
                    Pose(
                        Point(*np.append(self.scenario.robotPosition, 0)),
                        Quaternion(*STANDART_ORIENTATION),
                    ),
                    Pose(
                        Point(*np.append(self.scenario.robotGoal, 0)),
                        Quaternion(*STANDART_ORIENTATION),
                    ),
                )
                #
                # fill info dict
                if self.reset_count == 1:
                    info["new_scenerio_loaded"] = True
                else:
                    info["new_scenerio_loaded"] = False
                info["robot_goal_pos"] = self.scenario.robotGoal
                info["num_repeats_curr_scene"] = self.reset_count
                info[
                    "max_repeats_curr_scene"
                ] = 1000  # TODO: implement max number of repeats for scenario
            return info
            
        else:
            return "End"


def get_predefined_task(ns, mode="random", start_stage=1, PATHS=None):
    # type: (str, str, int, dict) -> None

    # get the map
    service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
    map_response = service_client_get_map()

    robot_manager = RobotManager(ns="", map_=map_response.map)
    obstacle_manager = ObstaclesManager(ns="", map_=map_response.map)
    pedsim_manager = PedsimManager()

    # Tasks will be moved to other classes or functions.
    task = None
    if mode == "scenario":
        rospy.set_param("/task_mode", "scenario")
        task = ScenarioTask(
            pedsim_manager, obstacle_manager, robot_manager, PATHS["scenario"]
        )

    if mode == "manual":
        rospy.set_param("/task_mode", "manual")
        task = ManualTask(pedsim_manager, obstacle_manager, robot_manager)

    return task
