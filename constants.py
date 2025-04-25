import sys
import math

"""This file gathers constant values used for different robots such as the lengths of the leg parts, the angle conventions, initial positions, etc.
Also, to maintain compatibility with legacy code, the user can chose whether the input and/or outputs of the kinematic functions are in (rads or degs) and (mm or meters).
Values in this file should however always be written in meters and in rads.
"""

class Constants:
    def __init__(self):

        self.DEFAULT_COMPUTE_IK_SIGN = -1
        self.BIOLOID = "BIOLOID"
        self.PHANTOMX = "PHANTOMX"
        self.PHANTOMX_SIMULATION = "PHANTOMX_SIMULATION"
        self.ARM_SIMULATION = "ARM_SIMULATION"
        self.AX12 = "AX12"
        self.MOTOR_TYPE = self.AX12
        self.ROBOT_TYPE = self.PHANTOMX_SIMULATION
        self.USING_SIMPLE_ROBOT = True
        self.USE_RADS_INPUT = False
        self.USE_RADS_OUTPUT = False
        self.USE_MM_INPUT = False
        self.USE_MM_OUTPUT = False

        self.constL1 = 0.001 * 54.8
        self.constL2 = 0.001 * 65.3
        self.constL3 = 0.001 * 133
        self.theta2Correction = 10.0
        self.theta2ExtraCorrection = 0
        self.theta3Correction = 33.0
        self.THETA3_MOTOR_SIGN = 1
        self.THETA2_MOTOR_SIGN = 1
        self.THETA1_MOTOR_SIGN = 1
        self.USE_RADS_INPUT = False
        self.USE_RADS_OUTPUT = False
        self.USE_MM_INPUT = True
        self.USE_MM_OUTPUT = True
        self.Z_DIRECTION = 1
        self.LEG_ANGLES = [
            math.pi / 4,
            -math.pi / 4,
            -math.pi / 2,
            -3 * math.pi / 4,
            3 * math.pi / 4,
            math.pi / 2,
        ]
        self.LIST_OF_INVERTED_IDS = [22, 23, 32, 33, 42, 43]
        self.LEG_CENTER_POS = [
            (0.1248, 0.06164, 0.001116),
            (0.1248, -0.06164, 0.001116),
            (0, -0.1034, 0.001116),
            (-0.1248, -0.06164, 0.001116),
            (-0.1248, 0.06164, 0.001116),
            (0, 0.1034, 0.001116),
        ]
        self.LEG_END_POS = [
            (0.2, 0, -0.05),
            (0.2, 0, -0.05),
            (0.2, 0, -0.05),
            (0.2, 0, -0.05),
            (0.2, 0, -0.05),
            (0.2, 0, -0.05),
        ]


    def update(self):
        # global constL1, constL2, constL3, theta2Correction, theta3Correction, \
        #     theta2ExtraCorrection, THETA3_MOTOR_SIGN, THETA2_MOTOR_SIGN, THETA1_MOTOR_SIGN,USE_RADS_INPUT, \
        #     USE_RADS_OUTPUT, USE_MM_INPUT, USE_MM_OUTPUT, Z_DIRECTION, LEG_ANGLES, LEG_END_POS, LEG_CENTER_POS
        # print("j'UPDATEEEEEEEEE")
        if self.ROBOT_TYPE == self.PHANTOMX:
            self.constL1 = 0.001 * 54.8
            self.constL2 = 0.001 * 65.3
            self.constL3 = 0.001 * 133
            self.theta2Correction = 10.0
            self.theta2ExtraCorrection = 0
            self.theta3Correction = 33.0
            self.THETA3_MOTOR_SIGN = 1
            self.THETA2_MOTOR_SIGN = 1
            self.THETA1_MOTOR_SIGN = 1
            self.USE_RADS_INPUT = False
            self.USE_RADS_OUTPUT = False
            self.USE_MM_INPUT = True
            self.USE_MM_OUTPUT = True
            self.Z_DIRECTION = 1
            self.LEG_ANGLES = [
                math.pi / 4,
                -math.pi / 4,
                -math.pi / 2,
                -3 * math.pi / 4,
                3 * math.pi / 4,
                math.pi / 2,
            ]
            self.LEG_END_POS = [
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
            ]
            self.LEG_CENTER_POS = [
                (0.1248, 0.06164, 0.001116),
                (0.1248, -0.06164, 0.001116),
                (0, -0.1034, 0.001116),
                (-0.1248, -0.06164, 0.001116),
                (-0.1248, 0.06164, 0.001116),
                (0, 0.1034, 0.001116),
            ]
            self.DEFAULT_COMPUTE_IK_SIGN = 1
            # self.LIST_OF_INVERTED_IDS = [22, 23, 32, 33, 42, 43]
        elif self.ROBOT_TYPE == self.BIOLOID:
            self.constL1 = 0.001 * 51
            self.constL2 = 0.001 * 63.7
            self.constL3 = 0.001 * 93
            # Angle to match the theory with reality for theta 2 (measures of the triangle are 22.5, 60.7, 63.7). => Angle =  -20.69
            self.theta2Correction = -20.69 * math.pi / 180.0
            # Same goes for theta 3 : +90 - 20.69 - a. Where a = asin(8.2/93) = 5.06
            self.theta3Correction = (90 + self.theta2Correction - 5.06) * math.pi / 180.0
            self.THETA3_MOTOR_SIGN = -1
            self.THETA2_MOTOR_SIGN = 1
            self.THETA1_MOTOR_SIGN = 1
            self.USE_RADS_INPUT = False
            self.USE_RADS_OUTPUT = False
            self.USE_MM_INPUT = True
            self.USE_MM_OUTPUT = True
            self.Z_DIRECTION = -1
            self.LEG_ANGLES = [math.pi / 2, 0, 0, -math.pi / 2, math.pi, math.pi]
        elif self.ROBOT_TYPE == self.PHANTOMX_SIMULATION:
            self.constL1 = 0.054
            self.constL2 = 0.0645
            # constL3 = 0.155  # Approximate value (different from real robot and couldn't find value in URDF yet)
            self.constL3 = 0.1602  # Approximate value (different from real robot and couldn't find value in URDF yet)
            self.theta2Correction = -16.0 * math.pi / 180.0  # Measured on real robot
            self.theta3Correction = (
                # -43.76 * math.pi / 180.0 + theta2Correction
                    -47 * math.pi / 180.0 + self.theta2Correction
            )  # Measured on real robot
            self.THETA3_MOTOR_SIGN = -1
            self.THETA2_MOTOR_SIGN = 1
            self.THETA1_MOTOR_SIGN = 1
            self.USE_RADS_INPUT = True
            self.USE_RADS_OUTPUT = True
            self.USE_MM_INPUT = False
            self.USE_MM_OUTPUT = False
            self.Z_DIRECTION = 1
            self.LEG_ANGLES = [
                -math.pi / 4,
                math.pi / 4,
                math.pi / 2,
                3 * math.pi / 4,
                -3 * math.pi / 4,
                -math.pi / 2,
            ]
            self.LEG_CENTER_POS = [
                (0.1248, -0.06164, 0.001116),
                (0.1248, 0.06164, 0.001116),
                (0, 0.1034, 0.001116),
                (-0.1248, 0.06164, 0.001116),
                (-0.1248, -0.06164, 0.001116),
                (0, -0.1034, 0.001116),
            ]
            self.LEG_END_POS = [
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
                (0.2, 0, -0.05),
            ]
        elif self.ROBOT_TYPE == self.ARM_SIMULATION:
            self.constL1 = 0.085
            self.constL2 = 0.185
            self.constL3 = 0.250
            self.theta2Correction = 0
            self.theta3Correction = 0
            self.THETA3_MOTOR_SIGN = 1
            self.THETA2_MOTOR_SIGN = 1
            self.THETA1_MOTOR_SIGN = 1
            self.USE_RADS_INPUT = True
            self.USE_RADS_OUTPUT = True
            self.USE_MM_INPUT = False
            self.USE_MM_OUTPUT = False
            self.Z_DIRECTION = 1
        else:
            print("ERROR: Unknwon ROBOT_TYPE '{}'".format(self.ROBOT_TYPE))
            sys.exit()