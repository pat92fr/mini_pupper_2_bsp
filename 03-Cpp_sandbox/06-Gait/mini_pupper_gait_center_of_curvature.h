#ifndef MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED
#define MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED

namespace mini_pupper
{
/*
self.rotate = abs(w_speed_dps) > 5.0 # threshold 5°/s
		self.CoM_CoC_distance_m = 0.0
		self.CoM_CoC_angle_rad = 0.0
		if self.rotate:
			self.CoM_CoC_distance_m = abs(self.CoM_linear_speed_mps/math.radians(w_speed_dps))
			if w_speed_dps > 0:
				self.CoM_CoC_angle_rad = math.atan2(y_speed_mps,x_speed_mps)+math.pi/2
			else:
				self.CoM_CoC_angle_rad = math.atan2(y_speed_mps,x_speed_mps)-math.pi/2

			# compute CoC position in the XY plane centered on CoM of body
			self.CoC = np.array(
				[
					self.CoM_CoC_distance_m*math.cos(self.CoM_CoC_angle_rad),
					self.CoM_CoC_distance_m*math.sin(self.CoM_CoC_angle_rad),
					0.0
				]
			)
			self.CoC = np.expand_dims(self.CoC, axis=1)
			#print("CoC:"+str(self.CoC))

			#print("Rotate velocity")
			self.ALL_mean_speed_mps = (self.CoM_CoC_distance_m+0.25)*math.radians(w_speed_dps) # distance from CoM to LEG
		else:
			#print("Inline velocity")
			self.ALL_mean_speed_mps = self.CoM_linear_speed_mps

*/
};

#endif // MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED
