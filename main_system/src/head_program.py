#!/usr/bin/env python
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
#from math import dist
try:
  from math import tau
except: # For Python 2 compatibility
  from math import pi
  tau = 2.0*pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

import serial # you need to install the pySerial :pyserial.sourceforge.net
import time
import codecs

#connect with the arduino (in windows, this is comport)
arduino = serial.Serial('/dev/ttyUSB0', 9600)

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    #x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    #x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    #d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    #cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    #return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    ## initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information of the robot

    # Name of the reference frame for the ur10:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # Print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # Get a list of all the groups in the ur10:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Print the entire state of the ur10
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self, angles):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = angles[0]
    joint_goal[1] = angles[1]
    joint_goal[2] = angles[2]
    joint_goal[3] = angles[3]
    joint_goal[4] = angles[4]
    joint_goal[5] = angles[5]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.001)

  def joint_6_move(self):

      #rotate the 5th joint with -90 degrees
      joint_goal = self.move_group.get_current_joint_values()
      print(joint_goal)
      joint_goal[0] = joint_goal[0]
      joint_goal[1] = joint_goal[1]
      joint_goal[2] = joint_goal[2]
      joint_goal[3] = joint_goal[3]
      joint_goal[4] = joint_goal[4]-0.5*pi
      joint_goal[5] = joint_goal[5]

      # The go command can be called with joint values, poses, or without any
      # parameters if you have already set the pose or joint target for the group
      self.move_group.go(joint_goal, wait=True)

      # Calling ``stop()`` ensures that there is no residual movement
      self.move_group.stop()

      # For testing:
      current_joints = self.move_group.get_current_joint_values()
      return all_close(joint_goal, current_joints, 0.05)

  def go_to_pose_goal(self, destination, orientation):
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = destination[0]
    pose_goal.position.y = destination[1]
    pose_goal.position.z = destination[2]

    quaternion = quaternion_from_euler(orientation[0],orientation[1],orientation[2], axes='sxyz')  # static/fixed frame XYZ

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    self.move_group.set_pose_target(pose_goal)
    self.move_group.allow_replanning(True)

    ## Call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)

    # Call `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.05)

  def plan_cartesian_path(self,new_position ,scale=1):
    move_group = self.move_group

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z += new_position[2]

    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += new_position[0]  # Second move forward/backwards in (x)
    wpose.position.y += new_position[1] # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 0.01 cm
    # which is why we will specify 0.0001 as the eef_step in Cartesian
    (plan, fraction) = move_group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.0001,        # eef_step
                                 0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):
    move_group = self.move_group
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    #print("===========================, the plan is: \n", plan)
    #input("execute plan")
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def init_objects(self, timeout=4):
    #place table and ground in RVIZ for planning
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.orientation.w = -0.8
    table_pose.pose.position.x = -0.8
    table_pose.pose.position.y = -0.15
    table_pose.pose.position.z = -0.43
    table_name = "table"
    self.scene.add_box(table_name, table_pose, size=(2,0.7,0.84))

    ground_pose = geometry_msgs.msg.PoseStamped()
    ground_pose.header.frame_id = "world"
    ground_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = 0.0
    ground_pose.pose.position.y = 0.0
    ground_pose.pose.position.z = -0.86
    ground_name = "ground"
    self.scene.add_box(ground_name, ground_pose, size=(3.0,3.0,0.001))

    bbb_name = "bbb"
    bba_name = "bba"
    bbc_name = "bbc"
    self.bba_name = bba_name
    self.bbb_name = bbb_name
    self.bbc_name = bbc_name

  def place_shelf(self, timeout=4):
    self.bbb_pose = geometry_msgs.msg.PoseStamped()
    self.bbb_pose.header.frame_id = "world"
    self.bbb_pose.pose.orientation.w = 1.0
    self.bbb_pose.pose.position.x = 1.1
    self.bbb_pose.pose.position.y = 0
    self.bbb_pose.pose.position.z = 0
    self.bbb_name = "bbb"
    self.scene.add_box(self.bbb_name, self.bbb_pose, size=(0.5,1.0,0.001))

    self.bba_pose = geometry_msgs.msg.PoseStamped()
    self.bba_pose.header.frame_id = "world"
    self.bba_pose.pose.orientation.w = 1.0
    self.bba_pose.pose.position.x = 1
    self.bba_pose.pose.position.y = 0.4
    self.bba_pose.pose.position.z = 0
    self.bba_name = "bba"
    self.scene.add_box(self.bba_name, self.bba_pose, size=(1.3,0.05,1))

    self.bbc_pose = geometry_msgs.msg.PoseStamped()
    self.bbc_pose.header.frame_id = "world"
    self.bbc_pose.pose.orientation.w = 1.0
    self.bbc_pose.pose.position.x = 1
    self.bbc_pose.pose.position.y = -0.4
    self.bbc_pose.pose.position.z = 0
    self.bbc_name = "bbc"
    self.scene.add_box(self.bbc_name, self.bbc_pose, size=(1.3,0.05,1))
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def place_shelf_side(self, timeout=4):
    #bottem shelf of the rack
    self.bbb_pose = geometry_msgs.msg.PoseStamped()
    self.bbb_pose.header.frame_id = "world"
    self.bbb_pose.pose.orientation.w = 1.0
    self.bbb_pose.pose.position.x = 0.075
    self.bbb_pose.pose.position.y = 1.5
    self.bbb_pose.pose.position.z = -0.76
    self.bbb_name = "bbb"
    self.scene.add_box(self.bbb_name, self.bbb_pose, size=(.9,.9,0.05))

    #upper shelf of the rack
    self.bbu_pose = geometry_msgs.msg.PoseStamped()
    self.bbu_pose.header.frame_id = "world"
    self.bbu_pose.pose.orientation.w = 1.0
    self.bbu_pose.pose.position.x = 0.075
    self.bbu_pose.pose.position.y = 1.5
    self.bbu_pose.pose.position.z = -0.21
    self.bbu_name = "bbu"
    self.scene.add_box(self.bbu_name, self.bbu_pose, size=(.9,.9,0.05))

    #sides of the shelf
    self.bba_pose = geometry_msgs.msg.PoseStamped()
    self.bba_pose.header.frame_id = "world"
    self.bba_pose.pose.orientation.w = 1.0
    self.bba_pose.pose.position.x = -0.375
    self.bba_pose.pose.position.y = 1.5
    self.bba_pose.pose.position.z = -0.11
    self.bba_name = "bba"
    self.scene.add_box(self.bba_name, self.bba_pose, size=(0.05,.9,1.5))

    self.bbc_pose = geometry_msgs.msg.PoseStamped()
    self.bbc_pose.header.frame_id = "world"
    self.bbc_pose.pose.orientation.w = 1.0
    self.bbc_pose.pose.position.x = 0.525
    self.bbc_pose.pose.position.y = 1.5
    self.bbc_pose.pose.position.z = -0.11
    self.bbc_name = "bbc"
    self.scene.add_box(self.bbc_name, self.bbc_pose, size=(0.05,.9,1.5))
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_stl(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    stl_pose = geometry_msgs.msg.PoseStamped()
    stl_pose.header.frame_id = "ee_link"

    #quaternion = quaternion_from_euler(-0.5*pi,pi,0, axes='sxyz')  # coplex end effector
    quaternion = quaternion_from_euler(0,pi,-0.5*pi, axes='sxyz')  # simple

    stl_pose.pose.orientation.x = quaternion[0]
    stl_pose.pose.orientation.y = quaternion[1]
    stl_pose.pose.orientation.z = quaternion[2]
    stl_pose.pose.orientation.w = quaternion[3]

    #stl_pose.pose.position.x = .205
    #stl_pose.pose.position.y = -0.115 #complex end effector
    #stl_pose.pose.position.z = -0.07

    stl_pose.pose.position.x = 0.0
    stl_pose.pose.position.y = -0.05 #sinple end effector
    stl_pose.pose.position.z = 0.04

    self.stl_name = "stl"

    #change this to the path for your PC
    self.scene.add_mesh(self.stl_name,stl_pose,'/home/ezra/catkin_ws/src/main_system/src/mesh/endeffectorabstract.STL', size=(0.001, 0.001, 0.001,0.001))

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_stl(self, timeout=4):
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the UR10's wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box.

    grasping_group = 'manipulator'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.stl_name, touch_links=touch_links)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def add_box(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ee_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = .075
    box_pose.pose.position.z = 0.0
    self.box_name = "box"

    self.scene.add_box(self.box_name, box_pose, size=(0.15, 0.025, 0.025))

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=4):
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the UR10's wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box.

    grasping_group = 'manipulator'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Remove the object from the planning scene:
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Remove the box from the world.
    self.scene.remove_world_object(self.box_name)

    ## **Note:** The object must be detached before we can remove it from the world

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def init_upright_path_constraints(self):
    #pose = geometry_msgs.msg.PoseStamped()
    pose = self.move_group.get_current_pose()
    self.upright_constraints = Constraints()
    self.upright_constraints.name = "upright"
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"
    print(pose.pose.orientation)
    orientation_constraint.link_name = self.eef_link
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    #orientation_constraint.absolute_z_axis_tolerance = 0.4
    orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
    orientation_constraint.weight = 1

    print("ratata, init klaar")
    self.upright_constraints.orientation_constraints.append(orientation_constraint)

  def enable_upright_path_constraints(self):
    self.move_group.set_path_constraints(self.upright_constraints)
    print("Lekker hutsen met constraint")

  def disable_upright_path_constraints(self):
    self.robot.set_path_constraints(None)

#=============START VISION===========================================================================
  def vision(self):
    print("1")
    print("2")
    print("3")
    x,y,z = -0.135, .4, -0.54
    x2,y2,z2 = 1.0, 0.0, 0.2

    #destination = [[x,y,z], [x2,y2,z2]]
    destination = [[x,y,z]]
    print('destination = ', destination)
    return destination

#==========================================================================================================

  def onOffFunction(self,command):
    time.sleep(2)
    if command =="on":
        print("The LED is on...")
        time.sleep(1)
        arduino.write(str.encode('L'))
    elif command =="off":
        print("The LED is off...")
        time.sleep(1)
        arduino.write(str.encode('H'))
    elif command =="bye":
        print("conection lost")
        time.sleep(1)
        arduino.close()
    else:
        print("Erros")

  def vacume(self):
    time.sleep(2)
    for i in range(20):
        vacume = arduino.readline()
        vacume = int(vacume.decode(encoding='UTF-8',errors='ignore'))
        vacume = vacume + vacume
        print(vacume)
    tvacume = vacume/20
    print(tvacume)
    if tvacume > 30:
        return True
    else:
        return False

  def sucking_system(self, status):
      #only for test purpose
      if status == True:
          sucking = 1
          print("sucking on")
      if status == False:
          sucking = 0
          print("sucking off")
      if sucking == 1:
          print("vacuum True")
          vacuum = True
      if sucking == 0:
          print("Vaccum False")
          vacuum = False
      return vacuum

  def quit(self):
    arduino.close() #close the connection with the arduino



#==============HEAD FUNCTIONS================================================================
  def top(self, amount_top):
      self.amount_top = amount_top
      print(self.amount_top)

      print("Look location")
      goal = [-1.2366639760050704, -3.3107352079934733, -1.8473003610545498, 0.445423184989181, -1.5702576591461013, 2.806662295774252]
      self.go_to_joint_state(goal)

      input("next") #after this statement, the robot is going to lower the end-effector to go further

      #look location with pose_goal
      #destination = [-0.2, .4, -0.45]
      #orientation = [-0.5*pi, -0.5*pi, -1*pi]
      #self.go_to_pose_goal(destination, orientation)

      new_location = [0.0,0.0,-0.15] #links/rechts, vooruit/achteruit, omhoog/omlaag
      cartesian_plan, fraction = self.plan_cartesian_path(new_location)
      self.execute_plan(cartesian_plan)

      joint_goal = self.move_group.get_current_joint_values()
      print("the postion is: ", joint_goal)
      further = input("Press Y to contiune: ")

      if further != "Y":
          sys.exit()
      else:
          print("next...")


      #enable vision
      location = self.vision()
      print("vision done")

      i = 0
      position = True

      multiple_boxes = 1
      #for i in range (len(location)): #could be used if the vision system is implemented
      while multiple_boxes ==1:
          #Go to the coordinates of the
          #orientation = [0, -1*pi, pi]
          x_vision = float(input("x: "))
          y_vision = float(input("y: "))   #cooridantes from vision
          z_vision = float(input("z: "))

          x = x_vision/1000
          y = z_vision/1000                #make the coordiantes from vision
          z = (y_vision/-1000) -0.15       #suitable for moveit

          new_location = [x,y, z ] #move in box

          goal = [-1.1568090103767112, -2.9555113618271966, -2.1667234177720793, -1.1610026218705551, -1.1561947900365714, 3.1419802669984445]
          self.go_to_joint_state(goal)

          #move to look position
          cartesian_plan, fraction = self.plan_cartesian_path(new_location)
          self.execute_plan(cartesian_plan)

          self.onOffFunction("on")

          """
          enable vacume
          """

          #loop for going further to pickup package
          down = 1
          while down == 1:
            new_location = [0,0,-0.02 ] #links/rechts, vooruit/achteruit, omhoog/omlaag
            cartesian_plan, fraction = self.plan_cartesian_path(new_location)
            self.execute_plan(cartesian_plan)
            down = int(input("press '1', to go down"))

          new_location = [0,0,0.05 ] #links/rechts, vooruit/achteruit, omhoog/omlaag
          cartesian_plan, fraction = self.plan_cartesian_path(new_location)
          self.execute_plan(cartesian_plan)

          if position == True:
              picked = self.sucking_system(True)
              #self.add_box()
              #self.attach_box()

          further = input("Press Y to contiune: ")
          if further != "Y":
              sys.exit()
          else:
              print("further")

          new_location = [0.0,-0.36,0.0] #links/rechts, vooruit/achteruit, omhoog/omlaag
          #move out the box
          cartesian_plan, fraction = self.plan_cartesian_path(new_location)
          self.execute_plan(cartesian_plan)

          #self.init_upright_path_constraints()
          #self.enable_upright_path_constraints()


          if picked == True:
              new_location = [-0.3,0.0,0.5] #links/rechts, vooruit/achteruit, omhoog/omlaag
              cartesian_plan, fraction = self.plan_cartesian_path(new_location)
              self.execute_plan(cartesian_plan)

              input("move joint 6 \n")
              self.joint_6_move()

              new_location = [-0.3,0.0,0.0] #links/rechts, vooruit/achteruit, omhoog/omlaag
              cartesian_plan, fraction = self.plan_cartesian_path(new_location)
              self.execute_plan(cartesian_plan)

              new_location = [0.0,-0.3,0.0] #links/rechts, vooruit/achteruit, omhoog/omlaag
              cartesian_plan, fraction = self.plan_cartesian_path(new_location)
              self.execute_plan(cartesian_plan)

              destination = [-1.0, 0.0, 0.]
              orientation = [pi, -1*pi, 0]
              #dropoff location
              self.go_to_pose_goal(destination, orientation)

              self.onOffFunction("off")
              """
              disable vacume
              """

              further = input("Press Y to contiune: ")
              if further != "Y":
                  sys.exit()
              else:
                  print("Abbort")

              if position == True:
                  #self.detach_box()
                  #self.remove_box()
                  picked = self.sucking_system(False)
          #go in loop for extra package
          multiple_boxes = int(input("press 1 for extra box: "))

  def bottom(self, amount):
      print(amount)

def main():
  try:
    print("")
    print("Press Ctrl-D to exit at any time")
    print("")

    head_program = MoveGroupPythonInterface()
    head_program.onOffFunction("off")

    head_program.init_objects()
    head_program.place_shelf_side()

    head_program.add_stl()
    head_program.attach_stl()

    amount_top = input("============ give the amount of boxes you want to pick up from the top layer ...")
    amount_bottom= input("============ give the amount of boxes you want to pick up from the bottom layer ...")

    head_program.top(amount_top)
    head_program.bottom(amount_bottom)
    #head_program.quit()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
