#!/usr/bin/env python
#
# ** task_scheduler_student.py **
# This task scheduler allows to take over the control from the task_scheduler, 
# and re-define the task programatically.
#
# Authors: Jun Zhang <jun.zhang@tugraz.at> 
#          Jesus Pestana Puerta <jesus.pestana@pro2future.at>
# Date: October 2024

from copy import deepcopy
import sys, os, getopt, json
import rospy, std_msgs.msg, geometry_msgs.msg
from icg_srvs.srv import SetPlannerWaypointParameter,SetWaypointParameter, SetAprilTagParameter, SetTrajectory4DParameter
import state_list as sl
import rospkg
from mav_planning_msgs.msg import PolynomialTrajectory4D
from mav_planning_msgs.msg import Point3D
from visualization_msgs.msg import Marker
from icg_msgs.msg import AprilTagDetection, AprilTagDetections


def usage():
  print(
    """
    ** task_scheduler_student.py **
    This task scheduler allows to take over the control from the task_scheduler, 
      and re-define the task programatically.

    Parameters: TBD (To Be Defined)

    Usage example:
      TBD 
    """
    )

# ** helper function **
def strToBool(x):
    return x in ("True","true",True)
# -- helper function --

cb_period_sec = 0.10

class TaskSchedulerStudent:

  def __init__(self):
    # ** Subscribers **
    self.sub_ts_input = rospy.Subscriber( # ts: task scheduler
      name="task_scheduler/in", data_class=std_msgs.msg.Int32, \
      callback=self.sub_ts_input_cb, buff_size=10)
    self.sub_ts_output = rospy.Subscriber( # ts: task scheduler
      name="task_scheduler/out", data_class=std_msgs.msg.Int32, \
      callback=self.sub_ts_output_cb, buff_size=10)
    # ** Publishers **
    self.pub_ts_output = rospy.Publisher("task_scheduler/out", std_msgs.msg.Int32, queue_size=10)
    self.pub_ts_self_input = rospy.Publisher("task_scheduler/in", std_msgs.msg.Int32, queue_size=10)

    # [STUDENT TODO] You may need to add some state or command indicators
    # Variables related to timers
    self.ts_start_received = False
    self.planner_waypoint_reached = False

  # [STUDENT TODO] You may need to add some state or command indicators
  def sub_ts_input_cb(self, msg):
    if msg.data == sl.States.PLANNER_TRAJECTORY_PLANNED.value:
      rospy.loginfo("[task_scheduler_student][sub_ts_input_cb] Received: {}".format(str(sl.States(msg.data).name)) )
      self.planner_waypoint_reached = True

  def sub_ts_output_cb(self, msg):
    if msg.data == sl.States.TASK_SCHEDULER_2_START.value:
      rospy.loginfo("[task_scheduler_student][sub_ts_output_cb] Received: {}".format(str(sl.States(msg.data).name)) )
      self.ts_start_received = True

  def wait_connections(self):
    while self.pub_ts_output.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.loginfo("Waiting for subscriber to connect")
      rospy.sleep(0.5)

  def start(self):
    rospy.loginfo("[task_scheduler_student] Start coding {}!".format("here") )

    # // ------------------------------------[STUDENT TODO]----------------------------------------------- // #

    # [1] Wait for: TASK_SCHEDULER_2_START (from the main task scheduler), and publish to TSI: TASK_SCHEDULER_2_STARTED.
    #     Note: when testing in your machine without main task scheduler running, you could publish it through terminal.
    # [2] Look for Apriltag markers by puslishing APRILTAG_SEARCH_MARKERS.
    # [3] Retrieve waypoint and heading information from detected Apriltag markers
    # [4a] Go to the next waypoint by trajectory planning
    # [4b] Or publish to TSI: TASK_SCHEDULER_2_STOPPED, if the destination point is reached.
    
    # // ------------------------------------------------------------------------------------------------- // #


def main():

  print("** BEGIN [0] Input parameters **")
  # print("Input parameter values:")
  print("-- END   [0] Input parameters --")
  print("")

  # ** 'global' variables **
  l_failed_assertions = list()
  debug = False
  # -- 'global' variables --

  print("** BEGIN [1] Loading data **")
  print("-- END   [1] Loading data --")

  print("** BEGIN [2] Create ROSNode object **")
  task_scheduler_sm = TaskSchedulerStudent()
  task_scheduler_sm.wait_connections()
  print("-- END   [2] Create ROSNode object --")

  print("** BEGIN [3] rospy.spin() **")
  task_scheduler_sm.start()
  rospy.spin()
  print("-- END   [3] rospy.spin() --")

  l_failed_assertions = list()
  return l_failed_assertions

if __name__ == '__main__':
  print("** BEGIN: task_scheduler_student.py script **")
  rospy.init_node('task_scheduler_student', disable_signals=True)
  l_failed_assertions = main()
  if len(l_failed_assertions)>0:
    print('Script failed, error/assertion numbers: "{0}".'.format(l_failed_assertions))
    print("--   END: task_scheduler_student.py script --")
    sys.exit( l_failed_assertions[0] )
  print("--   END: task_scheduler_student.py script --")


