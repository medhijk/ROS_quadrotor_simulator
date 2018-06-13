#!/usr/bin/env python

from check_force import call_ft
from balsa_arm_move_group_python_interface import *
import rospy


class MediationLayer(object):
    
    def __init__(self):
        
        # Initialize rospy node
        rospy.init_node("mediation_layer", anonymous=True)
        rospy.loginfo("Mediation layer successfully started")
        rospy.Subscriber("/gazebo/ft_sensor_topic", Wrench, self.call_ft)
    
	def check_force_torque(self):
        self.flag = check_force.flag
        pass

	def moveit_execute(self):
        self.group = balsa_arm_move_group_python_interface.group()
        if check_force_torque.flag == 0:
            self.group.execute(plan1)
        else:
                print("Too much force/torque.. cannot execute plan")
                pass

if __name__ == "__main__":
    try:
        MediationLayer()
    except rospy.ROSInterruptException:
        pass