#!/usr/bin/env python


#Remove this: next to do is actually subscribe a message.. it is only there in a function now

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from geometry_msgs.msg import Point

class HabitualLayer(object):


    def __init__(self):
        
        # Initialize rospy node
        rospy.init_node("habitual_layer", anonymous=True)
        rospy.loginfo("Habitual layer successfully started")
        
        self.quad_point = Point()
        self.arm_point = Point()
        self.execution_status = ""

        self.communication()
        self.main()

    # Communication setup
    def communication (self):
	self.quad_habit_delib = rospy.Publisher("/quad/habitual_to_deliberative", String, queue_size=10)
	self.arm_habit_delib = rospy.Publisher("/arm/habitual_to_deliberative", String, queue_size=10)

	self.quad_delib_habit = rospy.Subscriber("/quad/deliberative_to_habitual", Point, self.quad_habitual_callback)
	self.arm_delib_habit = rospy.Subscriber("/arm/deliberative_to_habitual", Point, self.arm_habitual_callback)
		
		
    # Callback functions	
    def quad_habitual_callback(self, msg):
        self.quad_point = msg
	
    def arm_habitual_callback(self, msg):
        self.arm_point = msg
		
    # if mission successful, publish success string, else publish unsuccessful
    """
			current_joint_values = group.get_current_joint_values()
		    manipulator_move_group_python_interface.plan1()
			print "============ Waiting while RVIZ displays plan..."
		    rospy.sleep(5)
		    planned_joint_values = group.get_current_joint_values()
		    if (current_joint_values == planned_joint_values):
                return True
		    else:
		        self.state == 3 # couldn't find a plan for arm
    """
	

    def main(self):
        while not rospy.is_shutdown():
	    self.manipulator_move_group_python_interface()

	

    def manipulator_move_group_python_interface(self):
	  
        print "============ Starting move_group_python_interface setup"
	moveit_commander.roscpp_initialize(sys.argv)
	#rospy.init_node('manipulator_move_group_python_interface', anonymous=True)
	  
	robot = moveit_commander.RobotCommander()
	  
	scene = moveit_commander.PlanningSceneInterface()
	  
	group = moveit_commander.MoveGroupCommander("manipulator")
	  
	display_trajectory_publisher = rospy.Publisher(
	                                    '/move_group/display_planned_path',
	                                    moveit_msgs.msg.DisplayTrajectory,
	                                    queue_size=20)
	  
	  
        #Getting basic information
	  
	print "============ Waiting for RVIZ..."
	rospy.sleep(10)
	print "============ Starting move_group_python_interface "
	
	planning_frame = group.get_planning_frame()
	print "============ Reference frame: %s" % group.get_planning_frame()
	
	eef_link = group.get_end_effector_link()
	print "============ End effector: %s" % group.get_end_effector_link()
	
	group_names = robot.get_group_names()
	print "============ Robot Groups:"
	print robot.get_group_names()
	
	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"
	
	"""
	  # Planning to a joint_space goal
	  
	  group_variable_values = group.get_current_joint_values()
	  print "============ Joint values: ", group_variable_values
	  
	  group_variable_values[0] = 0.1
	  group_variable_values[1] = 0.1
	  group_variable_values[2] = 0.1
	  group.set_joint_value_target(group_variable_values)
	  
	  plan1 = group.plan()
	  
	  print "============ Waiting while RVIZ displays plan..."
	  rospy.sleep(5)
	"""
	
	# Planning to a pose goal
	
	group_variable_values = group.get_current_pose(eef_link)
	
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.position.x = self.arm_point[0]
	pose_goal.position.y = self.arm_point[1]
	pose_goal.position.z = self.arm_point[2]
	group.set_pose_target(pose_goal)
	
	plan1 = group.plan()
	
	print "============ Waiting while RVIZ displays plan..."
	rospy.sleep(5)
	
	## Moving to a pose goal
	group.execute(plan1)
	
	
	# Check if goal has been reached
	executed_group_variable_values = group.get_current_pose(eef_link)
	
	if (group_variable_values == executed_group_variable_values):
	    self.execution_status = 1
	else:
	    self.execution_status = 0
	 
	 
	#########################################################################
	execution_msg = String()
	execution_msg.data = self.execution_status
	self.arm_habit_delib.publish(execution_msg)
	
	
	#########################################################################
	
	## When finished shut down moveit_commander.
	moveit_commander.roscpp_shutdown()
	
	## END_MOTION_PLANNING
	
	print "============ STOPPING"

if __name__=='__main__':
    try:
        HabitualLayer()
    except rospy.ROSInterruptException:
        pass
