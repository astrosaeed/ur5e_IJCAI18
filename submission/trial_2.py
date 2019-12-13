#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import time
#import gripper



class Ijcai18:

	def __init__(self):

		moveit_commander.roscpp_initialize(sys.argv)    # initialize moveit_commander
		rospy.init_node('ijcai18',    # initialize node
				anonymous=True)	

		self.robot = moveit_commander.RobotCommander()  # Instantiate a RobotCommander object

		self.scene = moveit_commander.PlanningSceneInterface()   # This object is an interface to the world surrounding the robot

		group_name = "arm"  # Ask Vidisha , why not ur5e??
		self.group = moveit_commander.MoveGroupCommander(group_name)   # This object is an interface to one group of joints.


		####  is used later to publish trajectories for RViz to visualize
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
											   moveit_msgs.msg.DisplayTrajectory,
											   queue_size=20)


		# We can get the name of the reference frame for this robot:
		self.planning_frame = self.group.get_planning_frame()
		print "============ Reference frame: %s" % self.planning_frame

		# We can also print the name of the end-effector link for this group:
		self.eef_link = self.group.get_end_effector_link()
		print "============ End effector: %s" % self.eef_link

		# We can get a list of all the groups in the robot:
		self.group_names = self.robot.get_group_names()
		print "============ Robot Groups:", self.robot.get_group_names()

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print "============ Printing robot state"
		print self.robot.get_current_state()
		
		print "============ Joint values:"
		joint_goal = self.group.get_current_joint_values()

	def forward_kinematic(self):

		# We can get the joint values from the group and adjust some of the values:
		print "============ Joint values:"
		joint_goal = self.group.get_current_joint_values()
		print joint_goal


		#joint_goal[0] = -1.57
		#joint_goal[1] = -2.13
		#joint_goal[2] = -1.57
		joint_goal[3] = -2.2

		#joint_goal[4] = 1.57
		#joint_goal[5] = 0
		


		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.group.stop()

	def inversekinematics(self,px,py,pz,ox,oy,oz,ow):


		pose_goal = geometry_msgs.msg.Pose()

		pose_goal.position.x = px
		pose_goal.position.y = py
		pose_goal.position.z = pz

		pose_goal.orientation.x = ox
		pose_goal.orientation.y = oy
		pose_goal.orientation.z = oz
		# w = 1 is horizontal towards pc
		pose_goal.orientation.w = ow
		
		self.group.set_pose_target(pose_goal)
		self.group.set_planner_id("RRTConnectkConfigDefault")
		self.group.set_planning_time(5)
		self.group.set_max_acceleration_scaling_factor(0.1)
		self.group.set_max_velocity_scaling_factor(0.1)
		plan = self.group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement

		#self.group.execute(plan, wait=True)
		self.group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		self.group.clear_pose_targets()


	def cartesian(self,xhorizontal,yhorizontal, vertical):

		scale= 1

		waypoints = []
		wpose = self.group.get_current_pose().pose
		wpose.position.z += scale * vertical  # First move up (z)
		wpose.position.y += scale * yhorizontal  # and sideways (y)
		wpose.position.x += scale * xhorizontal  # and sideways (y)
		waypoints.append(copy.deepcopy(wpose))
		self.group.set_max_acceleration_scaling_factor(0.5)
		self.group.set_max_velocity_scaling_factor(0.05)
		#wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		#waypoints.append(copy.deepcopy(wpose))

		#wpose.position.y -= scale * 0.1  # Third move sideways (y)
		#waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = self.group.compute_cartesian_path(
										   waypoints,   # waypoints to follow
										   0.01,        # eef_step
										   0.0)         # jump_threshold

		self.group.execute(plan, wait=True)



	def genCommand(self,char, command):
		"""Update the command according to the character entered by the user."""    
		
		if char == 'a':
			command = outputMsg.Robotiq2FGripper_robot_output();
			command.rACT = 1
			command.rGTO = 1
			command.rSP  = 255
			command.rFR  = 150

		if char == 'r':
			command = outputMsg.Robotiq2FGripper_robot_output();
			command.rACT = 0

		if char == 'c':
			command.rPR = 255

		if char == 'o':
			command.rPR = 0   

		#If the command entered is a int, assign this value to rPRA
		try: 
			command.rPR = int(char)
			if command.rPR > 255:
				command.rPR = 255
			if command.rPR < 0:
				command.rPR = 0
		except ValueError:
			pass                    
			
		if char == 'f':
			command.rSP += 25
			if command.rSP > 255:
				command.rSP = 255
				
		if char == 'l':
			command.rSP -= 25
			if command.rSP < 0:
				command.rSP = 0

				
		if char == 'i':
			command.rFR += 25
			if command.rFR > 255:
				command.rFR = 255
				
		if char == 'd':
			command.rFR -= 25
			if command.rFR < 0:
				command.rFR = 0

		return command
		

	def askForCommand(self,command):
		"""Ask the user for a command to send to the gripper."""    

		currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
		currentCommand += '  rACT = '  + str(command.rACT)
		currentCommand += ', rGTO = '  + str(command.rGTO)
		currentCommand += ', rATR = '  + str(command.rATR)
		currentCommand += ', rPR = '   + str(command.rPR )
		currentCommand += ', rSP = '   + str(command.rSP )
		currentCommand += ', rFR = '   + str(command.rFR )


		print currentCommand

		strAskForCommand  = '-----\nAvailable commands\n\n'
		strAskForCommand += 'r: Reset\n'
		strAskForCommand += 'a: Activate\n'
		strAskForCommand += 'c: Close\n'
		strAskForCommand += 'o: Open\n'
		strAskForCommand += '(0-255): Go to that position\n'
		strAskForCommand += 'f: Faster\n'
		strAskForCommand += 'l: Slower\n'
		strAskForCommand += 'i: Increase force\n'
		strAskForCommand += 'd: Decrease force\n'
		
		strAskForCommand += '-->'

		return raw_input(strAskForCommand)
		#return raw_input(strAskForCommand)
	def grasp(self):
		"""Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
		#rospy.init_node('Robotiq2FGripperSimpleController')
		
		pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

		command = outputMsg.Robotiq2FGripper_robot_output();
		timeout = 3
		timeout_start = time.time()
		while time.time() < timeout_start + timeout:
		#rospy.sleep(0.5)
			#command = genCommand(askForCommand(command), command)            
			command = self.genCommand('r', command)
			pub.publish(command)
			rospy.sleep(1)
			command = self.genCommand('a', command)
			pub.publish(command)
			rospy.sleep(1)
			command = self.genCommand('c', command)
			pub.publish(command)
			rospy.sleep(1)
							
	def ungrasp(self):
		"""Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
		#rospy.init_node('Robotiq2FGripperSimpleController')
		
		pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

		command = outputMsg.Robotiq2FGripper_robot_output();
		timeout = 2.5
		timeout_start = time.time()
		while time.time() < timeout_start + timeout:
		#rospy.sleep(0.5)
			#command = genCommand(askForCommand(command), command)            
			command = self.genCommand('r', command)
			pub.publish(command)
			rospy.sleep(1)
			command = self.genCommand('a', command)
			pub.publish(command)
			rospy.sleep(1)
			#command = self.genCommand('c', command)
			#pub.publish(command)
			#rospy.sleep(1)
							

	def go_to_initial_pose(self):
	
		self.inversekinematics(0.11,0.64,0.49,-0.48,0.02,-0.018,0.87)
		

	def lift(self):
		#self.ungrasp()
		rospy.sleep(5)
		self.cartesian(0,0,-0.20)
		rospy.sleep(3)
		self.grasp()
		rospy.sleep(3)
		self.cartesian(0,0,0.20)
		#rospy.sleep(3)
		#self.cartesian(0,0,-0.20)
		#rospy.sleep(3)
		#self.ungrasp()

	def lower(self):
		rospy.sleep(3)
		self.cartesian(0,0,-0.08)
		
	def drop(self):
		self.ungrasp()

	def push(self):
		self.grasp()
		rospy.sleep(4)
		#self.cartesian(-0.1,0,0)
		#rospy.sleep(5)
		self.cartesian(-0.05,0,0)
		rospy.sleep(4)
		self.cartesian(0,0,-0.20)
		rospy.sleep(4)
		self.cartesian(+0.10,0,0)
		rospy.sleep(4)
		self.cartesian(0,0,0.20)
	def get_pose(self):

		return self.group.get_current_pose()

def main():

	a= Ijcai18()
	#a.inversekinematics(0.23,0.61,0.30,-0.68,0.05,-0.05,0.72)
	#a.forward_kinematic()
	#a.go_to_initial_pose()
	#a.lift() ## measure the values by the tape measure for lifting
	#rospy.sleep(5)
	#print a.get_pose()
	#rospy.sleep(3)
	#a.cartesian(0,0,0.18)
	a.ungrasp()
	a.lift()
	a.lower()
	a.drop()
if __name__ == '__main__':
	main()