#!/usr/bin/env python

from check_force import call_ft
from balsa_arm_move_group_python_interface import *
import rospy


class DeliberativeLayer(object):
    
    def __init__(self):
        
        # Initialize rospy node
        rospy.init_node("deliberative_layer", anonymous=True)
        rospy.loginfo("Deliberative layer successfully started")
        rospy.Subscriber("/gazebo/ft_sensor_topic", Wrench, self.call_ft)
		
		
		self.state = 1
		self.goal_subplan = ""
		
		self.arm_limit_x = [-1000, 1000]
		self.arm_limit_y = [-1000, 1000]
		self.arm_limit_z = [-1000, 1000]
		
		self.quad_limit_x = [-1000, 1000]
		self.quad_limit_y = [-1000, 1000]
		self.quad_limit_z = [-1000, 1000]
		
		arm_waypoint=[]
		quad_waypoint=[]
    
	def main(self):
	    while not rospy.is_shutdown():
		    self.state_machine()
	
	
	#Run the State Machine
	
	def state_machine(self):
	
	    
		# Receive goals from operator
		if (self.state == 1):
		    # receive goal and forward
			received_goal = self.goal_received()
			if (received_goal == 1):
			    self.state = 2 # goal received for both UAV and arm
				self.goal_subplan = "quad arm planning"
			if (received_goal == 2):
			    self.state = 2 # goal received only for UAV
				self.goal_subplan = "quad planning"
			if (received_goal == 3):
			    self.state = 2 # goal received only for arm
				self.goal_subplan = "arm planning"
			else:
			    self.error = 1 # requested goals not received
				self.state = 0 # failed


		# Evaluate operator goals
		# Check if goals are in correct format		
		elif (self.state == 2):
		    format_quad = self.check_format_quad() # check if it's in a proper format ***outputs format=1 on success***
		    format_arm = self.check_format_arm() # check if it's in a proper format
			if (format_quad == 1 and format_arm == 1):
			    self.state = 3 # correct format of goals
			else:
			    self.error = 2 # incorrrect format for goals
				self.state = 0 # failed
		
		# Evaluate range of goals
		elif (self.state == 3):
				check_range_quad = self.check_range_quad()
				check_range_arm = self.check_range_arm()
				if (received_goal == 1):
				    if (check_range_quad == 1 and check_range_arm == 1):
					    range_goal = self.arm_workspace() # returns 1 if goal is possible
					else:
					    range_goal = 0 # waypoints out of range
			    if (received_goal == 2):
				    range_goal = check_range_quad # check if goal is within the range of quad
			    if (received_goal == 3):
				    range_goal = check_range_arm # check if end effector goal is within the range of arm
				    
				if (range_goal == 1):
				    self.state = 4
				else:
				    self.error = 3 # goals are out of range
					self.state = 0 # failed
		
		#Evaluate necessary subplan
		elif (self.state == 4)
		
		    if (self.goal_subplan == "quad planning"):
		        subplan = self.quad_plan_only(quad_waypoint) # only goals for quadrotor       # THIS FUNCTION MUST TAKE quad_waypoint AS ITS INPUT
		    elif (self.goal_subplan == "arm planning"):
		        arm_workspace = arm_workspace() # see if end effector goal is realizable
			    if (arm_workspace == 1):
			        subplan = self.arm_plan_only() # only goals for arm
			    else:
			        ##TODO
				    end_eff = end_effector_goal()
				    subplan = quad_plan_only(end_eff) # THIS FUNCTION MUST TAKE END_EFF AS ITS INPUT 
		    elif (self.goal_subplan == "quad arm planning"):
		        subplan_quad = self.quad_plan_only(quad_waypoint) # goals for both quadrotor and arm
				subplan_arm = self.arm_plan_only() # goals for both quadrotor and arm, execute one after another/together (if waypoint reached, execute this?)
		    else:
		        pass
		
		#Enact arm planning
		def arm_plan_only(self):
			    current_joint_values = group.get_current_joint_values()
		        manipulator_move_group_python_interface.plan1()
			    print "============ Waiting while RVIZ displays plan..."
			    rospy.sleep(5)
			    planned_joint_values = group.get_current_joint_values()
			    if (current_joint_values == planned_joint_values):
			        return True
			    else:
			        self.state == 3 # couldn't find a plan for arm
		
		#######################################################
		# TO DO 
		#
		# Map end effector from the quadcopter
		# if end effector has to go to position x,
		# quadcopter goal is (x, y, z+a) where 'a' is the length from quad center to end effector 
		#
		
		def end_effector_goal(self):
		    global arm_waypoint
			global arm_len
			end_eff[0] = arm_waypoint[0]  #end effector goal
			end_eff[1] = arm_waypoint[1]
			end_eff[2] = arm_waypoint[2] + arm_len
			return end_eff
		
		#######################################################
		
		
		
		
		def goal_received(self):
		    quad_waypoint_status = self.quad_waypoint_request() ###
			arm_waypoint_status = self.arm_waypoint_request()
			if (quad_waypoint_status == 1 and arm_waypoint_status == 1):
			    goal_received_status = 1 #both UAV and arm goals
			elif (quad_waypoint_status == 1 and arm_waypoint_status == 2):
			    goal_received_status = 2 #only UAV goals
			elif (quad_waypoint_status == 2 and arm_waypoint_status == 1):
			    goal_received_status = 3 #only arm goals
			elif (quad_waypoint_status == 2 and arm_waypoint_status == 2):
			    goal_received_status = 0 #no UAV or arm goals
			else:
			    goal_received_status = 0
			
			return goal_received_status
			
		def quad_waypoint_request(self):
			quad_goal = input ("Do you have a new goal for UAV? \nEnter 'y' or 'n': ").lower()
			if quad_goal == 'y':
			    global quad_waypoint
				
			    quad_x = float(input("enter quadrotor's goal: waypoint x-coordinate : "))
				quad_waypoint.append(quad_x)
				
			    quad_y = float(input("enter quadrotor's goal: waypoint y-coordinate : "))
				quad_waypoint.append(quad_y)
				
			    quad_z = float(input("enter quadrotor's goal: waypoint z-coordinate : "))
				quad_waypoint.append(quad_z)
			    
			    print("Requested waypoint for quadrotor is :", quad_waypoint)
				quad_waypoint_status = 1

			elif quad_goal == 'n':
			    print ("No goals requested for quadrotor")
				quad_waypoint_status = 2
				pass:
			else:
			    print ("Invalid response from operator")
				quad_waypoint_status = 0
			
			return quad_waypoint_status





		def arm_waypoint_request(self):
			arm_goal = input ("Do you have a new goal for arm? \nEnter 'y' or 'n': ").lower()
			if arm_goal == 'y':
			    global arm_waypoint
				
			    arm_x = float(input("enter arm's goal: waypoint x-coordinate : "))
				arm_waypoint.append(arm_x)
				
			    arm_y = float(input("enter arm's goal: waypoint y-coordinate : "))
				arm_waypoint.append(arm_y)
				
			    arm_z = float(input("enter arm's goal: waypoint z-coordinate : "))
				arm_waypoint.append(arm_z)
			    
			    print("Requested waypoint for arm is :", arm_waypoint)
				arm_waypoint_status = 1

			elif arm_goal == 'n':
			    print ("No goals requested for arm")
				arm_waypoint_status = 2
				pass:
			else:
			    print ("Invalid response from operator")
				arm_waypoint_status = 0
			
			return arm_waypoint_status






		
		def check_format_quad(self):
		    ###
			return 1
		

		def check_format_arm(self):
		    ###
			return 1


		def check_range_quad(self):
		    ###
			if (quad_waypoint[0] > self.quad_limit_x[0] and quad_waypoint[0] < self.quad_limit_x[1])
			    quad_range_x = True
			if (quad_waypoint[1] > self.quad_limit_y[0] and quad_waypoint[0] < self.quad_limit_y[1])
			    quad_range_y = True
			if (quad_waypoint[2] > self.quad_limit_z[0] and quad_waypoint[0] < self.quad_limit_z[1])
			    quad_range_z = True
			if quad_range_x and quad_range_y and quad_range_z:
			    return 1
				


		def check_range_arm(self):
		    
			
			###
			if (arm_waypoint[0] > self.arm_limit_x[0] and arm_waypoint[0] < self.arm_limit_x[1])
			    arm_range_x = True
			if (arm_waypoint[1] > self.arm_limit_y[0] and arm_waypoint[0] < self.arm_limit_y[1])
			    arm_range_y = True
			if (arm_waypoint[2] > self.arm_limit_z[0] and arm_waypoint[0] < self.arm_limit_z[1])
			    arm_range_z = True
			if (arm_range_x and arm_range_y and arm_range_z):
			    return 1




		def arm_workspace(self):
		    # TODO
			# function to check if the arm end effector goal is wthin the arm's reach/workspace 
			
			return 1
		    
			
			###
			if (arm_waypoint[0] > self.arm_limit_x[0] and arm_waypoint[0] < self.arm_limit_x[1])
			    arm_range_x = True
			if (arm_waypoint[1] > self.arm_limit_y[0] and arm_waypoint[0] < self.arm_limit_y[1])
			    arm_range_y = True
			if (arm_waypoint[2] > self.arm_limit_z[0] and arm_waypoint[0] < self.arm_limit_z[1])
			    arm_range_z = True
			if (arm_range_x and arm_range_y and arm_range_z):
			    return 1


		
		# or Enact quad planning
		def quad_plan_only:
		    #TODO
			return True
			
		
		# or Enact quad+arm planning
		def quad_arm_plan
		    #TODO
		
		#Send enact plan message to HL
	
	def check_force_torque(self):
        self.flag = check_force.flag
        pass

	def moveit_execute(self):
        self.group = balsa_arm_move_group_python_interface.group()
        if (check_force_torque.flag == 0):
            self.group.execute(plan1)
        else:
                print("Too much force/torque.. cannot execute plan")
                pass

if __name__ == "__main__":
    try:
        DeliberativeLayer()
    except rospy.ROSInterruptException:
        pass