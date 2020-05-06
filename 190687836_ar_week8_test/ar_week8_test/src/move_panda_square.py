#!/usr/bin/env python
			#code by abdul gaffar abdul khadeer
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import String
from std_msgs.msg import Float32





class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self,data):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.data=data


  def go_to_joint_state(self):

    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()





  def go_to_pose_goal(self):

    move_group = self.move_group


    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

  def plan_cartesian_path(self, scale=1):

    move_group = self.move_group
    data=self.data
    waypoints = []

    wpose = move_group.get_current_pose().pose
    #wpose.position.z -= scale *0.1 # First move up (z)
    wpose.position.y += data  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += data # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= data  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x -= data
    waypoints.append(copy.deepcopy(wpose))    
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction



  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher


    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);




  def execute_plan(self, plan):

    move_group = self.move_group

    move_group.execute(plan, wait=True)




def callback(data):
    rospy.loginfo(data.data)
    tutorial=MoveGroupPythonIntefaceTutorial(data.data)#create an object of the class
    tutorial.go_to_joint_state()#call this fucntion to move to joint state
    #tutorial.go_to_pose_goal()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()#call this function to plan
    tutorial.display_trajectory(cartesian_plan)#call this function to display trajectory
    rospy.sleep(5)#delay of five seconds after trajectory is shown
    tutorial.execute_plan(cartesian_plan)#excuet the displayed trajectory
    #cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    #tutorial.execute_plan(cartesian_plan)




def move_panda_square():
    rospy.init_node('move_panda_square', anonymous=True)
    rospy.Subscriber("square_size_generator1", Float32, callback)#subscrivbe to topic by node 1
    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  



if __name__ == '__main__':
    move_panda_square()
