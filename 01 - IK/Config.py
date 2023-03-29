import numpy as np

class Configuration:
    def __init__(self):

        #################### COMMANDS ####################
        self.max_x_velocity = 0.2
        self.max_y_velocity = 0.2
        self.max_yaw_rate = 2.0
        self.max_pitch = 20.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.01  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1.5

        ######################## GEOMETRY ######################
        self.LEG_OX = 0.059  # Leg origin along body X axis
        self.LEG_OY = 0.0235  # Leg origin along body Y axis
        self.LEG_OZ = 0.000  # Leg origin along body Z axis
        self.ABDUCTION_OFFSET = 0.026  # distance from abduction axis to leg
        self.LEG_LF = 0.050  # Length of femur (upper leg)
        self.LEG_LT = 0.060  # Length of tibia (lower leg)
        self.FOOT_RADIUS = 0.000

        self.LEG_MIN_LENGTH = 0.030
        self.LEG_MAX_LENGTH = 0.090

        self.LEG_ORIGINS = np.array(
            [
                [ self.LEG_OX, self.LEG_OX, -self.LEG_OX, -self.LEG_OX],
                [-self.LEG_OY, self.LEG_OY, -self.LEG_OY,  self.LEG_OY],
                [ self.LEG_OZ, self.LEG_OZ,  self.LEG_OZ,  self.LEG_OZ],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        #################### CROUCH ####################
        self.CROUCH_X =  0.059
        self.CROUCH_Y =  0.050        
        self.CROUCH_Z = -0.030
        self.CROUCH_X_SHIFT   = 0.000

        #################### STANCE ####################
        self.STANCE_X =  0.059
        self.STANCE_Y =  0.050        
        self.STANCE_Z = -0.080
        self.STANCE_X_SHIFT   = 0.000

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.03 #0.03
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.005 # 200Hz
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.12  # 0.09 duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.17  # 0.10 duration of the phase when only two feet are on the ground
        )
        
        ################## SWING ###########################

        self.MAX_JOINT_TORQUE = 0.3 # Nm

    @property
    def default_stance_with_zero_height(self):
        return np.array(
            [
                [
                     self.STANCE_X + self.STANCE_X_SHIFT,
                     self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                ],
                [
                    -self.STANCE_Y,
                    self.STANCE_Y,
                    -self.STANCE_Y,
                    self.STANCE_Y
                ],
                [0, 0, 0, 0],
            ]
        )

    @property
    def default_stance(self):
        return np.array(
            [
                [
                     self.STANCE_X + self.STANCE_X_SHIFT,
                     self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                ],
                [
                    -self.STANCE_Y,
                    self.STANCE_Y,
                    -self.STANCE_Y,
                    self.STANCE_Y
                ],
                [
                    self.STANCE_Z,
                    self.STANCE_Z,
                    self.STANCE_Z,
                    self.STANCE_Z
                ],
            ]
        )

    ### default_stance_STANCE_Z = config.default_stance + np.array([[0.0,0.0,config.STANCE_Z_ref],]*4).transpose()

    @property
    def default_crouch_with_zero_height(self):
        return np.array(
            [
                [
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                ],
                [
                    -self.CROUCH_Y,
                    self.CROUCH_Y,
                    -self.CROUCH_Y,
                    self.CROUCH_Y
                ],
                [
                    0,
                    0,
                    0,
                    0
                ],
            ]
        )

    @property
    def default_crouch(self):
        return np.array(
            [
                [
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                ],
                [
                    -self.CROUCH_Y,
                    self.CROUCH_Y,
                    -self.CROUCH_Y,
                    self.CROUCH_Y
                ],
                [
                    self.CROUCH_Z,
                    self.CROUCH_Z,
                    self.CROUCH_Z,
                    self.CROUCH_Z
                ],
            ]
        )

    ################## SWING ###########################

    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

