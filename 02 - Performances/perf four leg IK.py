from Config import Configuration
import Kinematics
import numpy as np
import math
import matplotlib.pyplot as plt
import time

##DEBUG #######################################################################

# TRACE macro
DEBUG = True
DEBUG = False

def log(s):
    if DEBUG:
        print(s)

##DEBUG #######################################################################


if __name__ == '__main__':

	config = Configuration()

	# define joint position
	target_joint_position = np.radians(np.array(
		[	#   FR,   FL,   RR,   RL
			[ 25.0, 10.0,-33.0,-15.0], # hips abduction revolute joints
			[200.0,160.0,190.0, 95.0], # hips flexion/extension revolute joints
			[120.0, 80.0, 70.0, 45.0]  # knee flexion/extension revolute joints
		]
	))
	log("target_joint_position:\n"+str(np.round(np.degrees(target_joint_position),1)))
	
	# compute FK from joint position to feet position in BRF
	target_feet_position_BRF = Kinematics.four_legs_forward_kinematics_BRF(target_joint_position,config)
	log("target_feet_position_BRF:\n"+str(np.round(target_feet_position_BRF,3)))

	t0 = int(time.time()*1000.0)

	for i in range(1000):

		# compute IK from feet position to joint position in BRF
		IK_explicit_joint_position = Kinematics.four_legs_explicit_inverse_kinematics_BRF(target_feet_position_BRF,config)
		log("IK_explicit_joint_position:\n"+str(np.round(np.degrees(IK_explicit_joint_position),1)))

	t1 = int(time.time()*1000.0)
	print("1000x four-leg FK+IK delay:"+str(t1-t0)+"ms")
	print("four-leg FK+IK frequency:"+str( 1000000.0/(t1-t0) )+"Hz")

