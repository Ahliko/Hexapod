import time
from enum import Enum

import pybullet as p
import pygame
import pypot.dynamixel
from onshape_to_robot.simulation import Simulation

from kinematics_empty import *
from scipy.spatial.transform import Rotation


class Parameters:
    def __init__(
            self,
            constants: Constants,
            freq=50,
            speed=1,
            z=-60,
            travelDistancePerStep=80,
            lateralDistance=90,
            frontDistance=87,
            frontStart=32,
            method="constantSpeed",
            maxAccel=6000,
            maxSpeed=500,
            startFromInit=True,
            endToInit=False,
            up=False,
            down=False,
            left=False,
            right=False,
            walkMagnitudeX=0,
            walkMagnitudeY=0,
            activateWalk=False,
    ):
        self.constants = constants
        self.freq = freq
        self.speed = speed
        self.z = z
        self.travelDistancePerStep = travelDistancePerStep
        self.lateralDistance = lateralDistance
        self.frontDistance = frontDistance
        self.frontStart = frontStart
        self.method = method
        self.maxAccel = maxAccel
        self.maxSpeed = maxSpeed
        self.startFromInit = startFromInit
        self.endToInit = endToInit
        self.up = up
        self.down = down
        self.left = left
        self.right = right
        self.walkMagnitudeX = walkMagnitudeX
        self.walkMagnitudeY = walkMagnitudeY
        self.activateWalk = activateWalk
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = self.constants.LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        if self.constants.ROBOT_TYPE == self.constants.BIOLOID:
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, -self.frontDistance]
            )
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, self.frontDistance]
            )
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, -self.frontDistance]
            )
            self.initLeg.append(
                [self.frontStart + self.travelDistancePerStep / 2, self.frontDistance]
            )
        elif self.constants.ROBOT_TYPE == self.constants.PHANTOMX or self.constants.ROBOT_TYPE == self.constants.PHANTOMX_SIMULATION:
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])
            self.initLeg.append([self.lateralDistance, 0])


# Classes used to use dxl_io directly and by-pass Pypot's Robot class
class SimpleMotor:
    def __init__(self, id):
        self.id = id
        self.present_position = 0
        self.goal_position = 0
        self.smooth_start_position = 0
        self.smooth_final_position = 0

    def __repr__(self):
        return f"id {self.id}, goal_position {self.goal_position}, present_position {self.present_position}"


class SimpleRobot:
    def __init__(self, dxl_io: pypot.dynamixel.DxlIO, constants: Constants):
        self.dxl_io = dxl_io
        # self.legs = {
        #     1: [SimpleMotor(8), SimpleMotor(10), SimpleMotor(12)],
        #     2: [SimpleMotor(14), SimpleMotor(16), SimpleMotor(18)],
        #     3: [SimpleMotor(13), SimpleMotor(15), SimpleMotor(17)],
        #     4: [SimpleMotor(7), SimpleMotor(9), SimpleMotor(11)],
        #     5: [SimpleMotor(1), SimpleMotor(3), SimpleMotor(5)],
        #     6: [SimpleMotor(2), SimpleMotor(4), SimpleMotor(6)],
        # }
        self.legs = {
            1: [SimpleMotor(11), SimpleMotor(12), SimpleMotor(13)],
            2: [SimpleMotor(21), SimpleMotor(22), SimpleMotor(23)],
            3: [SimpleMotor(31), SimpleMotor(32), SimpleMotor(33)],
            4: [SimpleMotor(41), SimpleMotor(42), SimpleMotor(43)],
            5: [SimpleMotor(51), SimpleMotor(52), SimpleMotor(53)],
            6: [SimpleMotor(61), SimpleMotor(62), SimpleMotor(63)],
        }
        self.delay_after_write = 0.01
        self.params = None
        self.constants = constants

    def __repr__(self):
        output = "##### Robot #####\n"
        for k, v in self.legs.items():
            output += "# Leg{}: [{}] [{}] [{}]\n".format(k, v[0], v[1], v[2])
        return output

    def print_dk(self, constants: Constants):
        output = "##### Robot #####\n"
        output += "/!\ The IK check only works for legs whose IDs are not in LIST_OF_INVERTED_IDS\n"

        for k, v in self.legs.items():
            p0, p1, p2, p3 = computeDKDetailed(
                theta1=v[0].present_position,
                theta2=v[1].present_position,
                theta3=v[2].present_position,
                constants=constants)
            output += "# Leg{}. Angles: [{:.2f}] [{:.2f}] [{:.2f}]. DK P3: x={:.2f}, y={:.2f}, z={:.2f} DK P2: x={:.2f}, y={:.2f}, z={:.2f}".format(
                k,
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
                p3[0],
                p3[1],
                p3[2],
                p2[0],
                p2[1],
                p2[2],
            )
            # This doens't work well because the low level will invert the sign of some motors if we ue the new "LIST_OF_INVERTED_IDS"
            # Testing if the IK is coherent with the DK
            angles_from_ik = computeIK(p3[0], p3[1], p3[2], constants=constants)
            output += f"\n IK({p3[0]:.2f}, {p3[1]:.2f}, {p3[2]:.2f}) --> theta1={angles_from_ik[0]:.2f} (real:{v[0].present_position:.2f})"
            output += (
                f", theta2={angles_from_ik[1]:.2f} (real:{v[1].present_position:.2f})"
            )
            output += (
                f", theta3={angles_from_ik[2]:.2f} (real:{v[2].present_position:.2f})\n"
            )

        print(output)

    def init(self, constants: Constants):
        """Sets the goal_position to the present_position"""
        self.tick_read(constants, verbose=True)
        for k, v in self.legs.items():
            v[0].goal_position = v[0].present_position
            v[1].goal_position = v[1].present_position
            v[2].goal_position = v[2].present_position

    def motors(self):
        list_of_motors = []
        for k, v in self.legs.items():
            list_of_motors.append(v[0])
            list_of_motors.append(v[1])
            list_of_motors.append(v[2])
        return list_of_motors

    def enable_torque(self, list_of_ids=None):
        to_send = []
        if list_of_ids == None:
            for k, v in self.legs.items():
                to_send.append(v[0].id)
                to_send.append(v[1].id)
                to_send.append(v[2].id)
        else:
            to_send = list_of_ids
        self.dxl_io.enable_torque(to_send)
        time.sleep(self.delay_after_write)

    def disable_torque(self, list_of_ids=None):
        to_send = []
        if list_of_ids == None:
            for k, v in self.legs.items():
                to_send.append(v[0].id)
                to_send.append(v[1].id)
                to_send.append(v[2].id)
        else:
            to_send = list_of_ids
        print("Disabling torques...")
        self.dxl_io.disable_torque(to_send)
        print("waiting...")
        time.sleep(self.delay_after_write)
        print(f"Torques disabled for ids: {list_of_ids}")

    def tick_read(self, constants: Constants, verbose=False):
        # Creating a list for a read request
        to_read = []
        for k, v in self.legs.items():
            to_read.append(v[0].id)
            to_read.append(v[1].id)
            to_read.append(v[2].id)

        if verbose:
            print("Sending read command '{}'".format(to_read))
        result = self.dxl_io.get_present_position(to_read)
        for i in range(len(to_read)):
            id = to_read[i]
            value = result[i]
            # TODO get a getter by ID instead of this loop spamming...
            for m in self.motors():
                factor = 1.0
                if m.id == id:
                    if id in constants.LIST_OF_INVERTED_IDS:
                        factor = -1.0
                    m.present_position = value * factor
        if verbose:
            print("Read tick done")

    def tick_write(self, constants: Constants, verbose=False):
        # Creating a dict for a write request
        to_write = {}
        for k, v in self.legs.items():
            for i in range(3):
                if self.constants.MOTOR_TYPE == self.constants.AX12:
                    # Not sending values out of motor range
                    if v[i].goal_position <= -150 or v[i].goal_position >= 150:
                        continue
                factor = 1.0
                if v[i].id in constants.LIST_OF_INVERTED_IDS:
                    factor = -1.0
                to_write[v[i].id] = v[i].goal_position * factor

        if verbose:
            print("Sending write command '{}'".format(to_write))
        self.dxl_io.set_goal_position(to_write)
        if verbose:
            print("Write tick done")
        time.sleep(self.delay_after_write)

    def tick_read_and_write(self, constants: Constants, verbose=False):
        # Creating a list for a read request and a dict for a write request
        try:
            self.tick_read(constants)
            self.tick_write(constants)
            if verbose:
                print("IO tick done")
        finally:
            pass

    def smooth_tick_read_and_write(self, delay, constants: Constants, verbose=False):
        # Reads the current state of the robot and applies the write positions smoothly over 'time'
        self.tick_read(constants)
        # Setting the start and end positions
        t0 = time.time()
        for m in self.motors():
            m.smooth_start_position = m.present_position
            m.smooth_final_position = m.goal_position
            if verbose:
                print(
                    "m.smooth_start_position {}, m.smooth_final_position {}".format(
                        m.smooth_start_position,
                        m.smooth_final_position,
                    )
                )
        t = time.time() - t0
        while t < delay:
            for m in self.motors():
                m.goal_position = (t / delay) * (
                        m.smooth_final_position - m.smooth_start_position
                ) + m.smooth_start_position
            self.tick_write(constants, verbose=verbose)
            t = time.time() - t0
        for m in self.motors():
            m.goal_position = m.smooth_final_position
        self.tick_write(constants, verbose=verbose)
        self.tick_read(constants)
        self.print_dk(constants=constants)
        if verbose:
            print("IO smooth tick done")


# Class used to simulate the robot in PyBullet's (actually onshape_to_robot) environnment
class SimpleRobotSimulation:
    def __init__(self, sim, constants: Constants):
        self.sim = sim  # e.g onshape_to_robot.simulation.Simulation("phantomx_description/urdf/phantomx.urdf", gui=True, panels=True, useUrdfInertia=False)
        self.legs = {
            1: [
                SimpleMotor("j_c1_rf"),
                SimpleMotor("j_thigh_rf"),
                SimpleMotor("j_tibia_rf"),
            ],
            6: [
                SimpleMotor("j_c1_rm"),
                SimpleMotor("j_thigh_rm"),
                SimpleMotor("j_tibia_rm"),
            ],
            5: [
                SimpleMotor("j_c1_rr"),
                SimpleMotor("j_thigh_rr"),
                SimpleMotor("j_tibia_rr"),
            ],
            2: [
                SimpleMotor("j_c1_lf"),
                SimpleMotor("j_thigh_lf"),
                SimpleMotor("j_tibia_lf"),
            ],
            3: [
                SimpleMotor("j_c1_lm"),
                SimpleMotor("j_thigh_lm"),
                SimpleMotor("j_tibia_lm"),
            ],
            4: [
                SimpleMotor("j_c1_lr"),
                SimpleMotor("j_thigh_lr"),
                SimpleMotor("j_tibia_lr"),
            ],
        }
        self.delay_after_write = 0.00
        self.drawOn = True
        self.params = None
        self.constants = constants
        self.centerCamera = False

    def __repr__(self):
        output = "##### Robot #####\n"
        for k, v in self.legs.items():
            output += "# Leg{}: [{}] [{}] [{}]\n".format(k, v[0], v[1], v[2])
        return output

    def print_dk(self, constants):
        output = "##### Robot #####\n"

        for k, v in self.legs.items():
            p0, p1, p2, p3 = computeDKDetailed(
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
                constants=constants)
            output += "# Leg{}. Angles: [{:.2f}] [{:.2f}] [{:.2f}]. DK P3: x={:.2f}, y={:.2f}, z={:.2f} DK P2: x={:.2f}, y={:.2f}, z={:.2f}\n".format(
                k,
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
                p3[0],
                p3[1],
                p3[2],
                p2[0],
                p2[1],
                p2[2],
            )
        print(output)

    def init(self, constants: Constants):
        """Sets the goal_position to the present_position"""
        self.tick_read(verbose=True, constants=constants)
        for k, v in self.legs.items():
            v[0].goal_position = v[0].present_position
            v[1].goal_position = v[1].present_position
            v[2].goal_position = v[2].present_position

    def motors(self):
        list_of_motors = []
        for k, v in self.legs.items():
            list_of_motors.append(v[0])
            list_of_motors.append(v[1])
            list_of_motors.append(v[2])
        return list_of_motors

    def enable_torque(self, list_of_ids=None):
        if list_of_ids == None:
            # Enabling torque for all motors (it's weird I know)
            self.sim.maxTorques = {}
        else:
            new_torques = {}
            for k, v in self.sim.maxTorques.items():
                if not (k in list_of_ids):
                    # Copying previous IDs and removing the ones that need their torque reactivated
                    new_torques[k] = v
            self.sim.maxTorques = new_torques

    def disable_torque(self, list_of_ids=None):
        if list_of_ids == None:
            # Disabling torque for all motors
            for m in self.motors():
                self.sim.maxTorques[m.id] = 0
        else:
            new_torques = self.sim.maxTorques
            for id in list_of_ids:
                if not (id in new_torques):
                    # Adding new IDs and avoiding duplicates
                    new_torques[id] = 0
            self.sim.maxTorques = new_torques

    def tick_read(self, constants: Constants, verbose=False):
        # Read and write are not separated for now in the simulator
        self.tick_read_and_write(constants=constants)

    def tick_write(self, constants: Constants, verbose=False):
        # Read and write are not separated for now in the simulator
        self.tick_read_and_write(constants=constants)

    def tick_read_and_write(self, constants, verbose=False):
        # Creating a list for a read request and a dict for a write request
        to_write = {}
        for k, v in self.legs.items():
            for i in range(3):
                to_write[v[i].id] = v[i].goal_position

        if verbose:
            print("Sending write command '{}'".format(to_write))
        state = self.sim.setJoints(to_write)
        for m in self.motors():
            id = m.id
            position = state[id][
                0
            ]  # contains [position, speed, (3 forces and 3 torques)
            m.present_position = position
        if self.drawOn:
            self.drawLegTips(constants=constants)

        if verbose:
            print("IO tick done")

    def smooth_tick_read_and_write(self, delay, constants: Constants, verbose=False):
        # Reads the current state of the robot and applies the write positions smoothly over 'time'
        self.tick_read(constants=constants)
        # Setting the start and end positions
        t0 = time.time()
        for m in self.motors():
            m.smooth_start_position = m.present_position
            m.smooth_final_position = m.goal_position
            if verbose:
                print(
                    "m.smooth_start_position {}, m.smooth_final_position {}".format(
                        m.smooth_start_position,
                        m.smooth_final_position,
                    )
                )
        t = time.time() - t0
        while t < delay:
            for m in self.motors():
                m.goal_position = (t / delay) * (
                        m.smooth_final_position - m.smooth_start_position
                ) + m.smooth_start_position
            self.tick_write(verbose=verbose, constants=constants)
            t = time.time() - t0
            # Blocking call has to tick the simulation here
            self.sim.tick()
        for m in self.motors():
            m.goal_position = m.smooth_final_position
        self.tick_write(verbose=verbose, constants=constants)
        if verbose:
            print("IO smooth tick done")

    def drawLegTips(self, constants, duration=2):
        # Printing the tip of each leg
        robot_pose = (
            self.sim.getRobotPose()
        )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        yaw = robot_pose[1][2]
        for i in range(6):
            motors = self.legs[i + 1]
            pos = computeDK(
                motors[0].present_position,
                motors[1].present_position,
                motors[2].present_position,
                use_rads=True,
                constants=constants)
            pos = rotaton_2D(
                pos[0],
                pos[1],
                pos[2],
                constants.LEG_ANGLES[i] + yaw,
            )
            leg_center_position = rotaton_2D(
                self.constants.LEG_CENTER_POS[i][0],
                self.constants.LEG_CENTER_POS[i][1],
                self.constants.LEG_CENTER_POS[i][2],
                yaw,
            )
            pos[0] += robot_pose[0][0] + leg_center_position[0]
            pos[1] += robot_pose[0][1] + leg_center_position[1]
            pos[2] += robot_pose[0][2] + leg_center_position[2]
            self.sim.addDebugPosition(pos, duration=duration)

    def tickSim(self):
        if self.centerCamera:
            self.centerCameraOnRobot()
        self.sim.tick()

    def centerCameraOnRobot(self):
        robot_pose = (
            self.sim.getRobotPose()
        )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        self.sim.lookAt(robot_pose[0])


def setAnglesToLeg(angles, leg, verbose=False):
    i = 0
    for m in leg:
        if verbose:
            print("Setting angle {} to id {}".format(angles[i], m.id))
        m.goal_position = angles[i]
        i = i + 1


def setPositionToLeg(x, y, z, leg, constants: Constants):
    angles = computeIK(x, y, z, constants=constants)
    setAnglesToLeg(angles, leg)


def setPositionToRobot(x: float, y: float, z: float, robot: SimpleRobot | SimpleRobotSimulation, params: Parameters,
                       constants: Constants,
                       extra_theta=0):
    angles = []
    for i in range(1, 7):
        # For the robot to move forward, the legs have to move backwards, hence the "-"
        angles.append(
            computeIKOriented(x=-x, y=-y, z=-z, leg_id=i, params=params, extra_theta=extra_theta, constants=constants))

    if constants.USING_SIMPLE_ROBOT:
        for i in range(1, 7):
            setAnglesToLeg(angles[i - 1], robot.legs[i], verbose=False)
    else:
        setAnglesToLeg(angles[0], robot.leg1)
        setAnglesToLeg(angles[1], robot.leg2)
        setAnglesToLeg(angles[2], robot.leg3)
        setAnglesToLeg(angles[3], robot.leg4)
        setAnglesToLeg(angles[4], robot.leg5)
        setAnglesToLeg(angles[5], robot.leg6)


def setPositionToTripod1(x, y, z, robot, params, constants: Constants):
    angles = []

    # For the robot to move forward, the legs have to move backwards, hence the "-"
    angles.append(computeIKOriented(-x, -y, -z, 1, params))
    angles.append(computeIKOriented(-x, -y, -z, 3, params))
    angles.append(computeIKOriented(-x, -y, -z, 5, params))

    if constants.USING_SIMPLE_ROBOT:
        setAnglesToLeg(angles[0], robot.legs[1])
        setAnglesToLeg(angles[1], robot.legs[3])
        setAnglesToLeg(angles[2], robot.legs[5])
    else:
        setAnglesToLeg(angles[0], robot.leg1)
        setAnglesToLeg(angles[1], robot.leg3)
        setAnglesToLeg(angles[2], robot.leg5)


def setPositionToTripod2(x, y, z, robot, params, constants: Constants):
    angles = []

    # For the robot to move forward, the legs have to move backwards, hence the "-"
    angles.append(computeIKOriented(-x, -y, -z, 4, params))
    angles.append(computeIKOriented(-x, -y, -z, 2, params))
    angles.append(computeIKOriented(-x, -y, -z, 6, params))

    if constants.USING_SIMPLE_ROBOT:
        setAnglesToLeg(angles[0], robot.legs[4])
        setAnglesToLeg(angles[1], robot.legs[2])
        setAnglesToLeg(angles[2], robot.legs[6])
    else:
        setAnglesToLeg(angles[0], robot.leg4)
        setAnglesToLeg(angles[1], robot.leg2)
        setAnglesToLeg(angles[2], robot.leg6)


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


def printAllMotors(robot):
    for m in robot.motors:
        print("ID '{}' position '{}'".format(m.id, m.present_position))


def allMotorsCompliantAndPrint(robot):
    for m in robot.motors:
        print("ID '{}' position '{}'".format(m.id, m.present_position))
        m.compliant = True


def allMotorsCompliantAndPrintForEver(robot):
    while True:
        time.sleep(0.1)
        for m in robot.motors:
            print("ID '{}' position '{}'".format(m.id, m.present_position))
            m.compliant = True


def allMotorsNotCompliant(robot):
    for m in robot.motors:
        m.compliant = False


def allMotorsCompliant(robot):
    for m in robot.motors:
        m.compliant = True


def allMotorsToPresentPosition(robot):
    for m in robot.motors:
        print("Setting motor {} to angle {}".format(m.id, m.present_position))
        m.goal_position = m.present_position


# def testPrimitive(robot, params):
#     walk = primitives.TestPrimitive(robot, params)
#     while True:
#         try:
#             method = input("Method (minJerk, constantSpeed, brutal) ? : ")
#             if len(method) < 2:
#                 method = "constantSpeed"
#             params.method = method
#         except ValueError:
#             print("default value used")
#             params.method = "constantSpeed"
#         walk.start()
#         walk.wait_to_stop()


def initPositionTripod(robot, constants: Constants, distance=90, z=-60):
    sqrt2 = math.sqrt(2)
    # Lateral arms
    anglesLeg1 = computeIK(90, 0, z, constants=constants)
    # anglesLeg4 = computeIK(90, 0, z)
    anglesLeg4 = computeIK(90, 0, z + 30, constants=constants)
    # Back and front arms
    anglesLeg2 = computeIK(distance / sqrt2, -distance / sqrt2, z, constants=constants)
    anglesLeg6 = computeIK(distance / sqrt2, distance / sqrt2, z, constants=constants)
    anglesLeg5 = computeIK(distance / sqrt2, -distance / sqrt2, z, constants=constants)
    anglesLeg3 = computeIK(distance / sqrt2, distance / sqrt2, z, constants=constants)

    if constants.USING_SIMPLE_ROBOT:
        setAnglesToLeg(anglesLeg1, robot.legs[1])
        setAnglesToLeg(anglesLeg2, robot.legs[2])
        setAnglesToLeg(anglesLeg3, robot.legs[3])
        setAnglesToLeg(anglesLeg4, robot.legs[4])
        setAnglesToLeg(anglesLeg5, robot.legs[5])
        setAnglesToLeg(anglesLeg6, robot.legs[6])
    else:
        setAnglesToLeg(anglesLeg1, robot.leg1)
        setAnglesToLeg(anglesLeg2, robot.leg2)
        setAnglesToLeg(anglesLeg3, robot.leg3)
        setAnglesToLeg(anglesLeg4, robot.leg4)
        setAnglesToLeg(anglesLeg5, robot.leg5)
        setAnglesToLeg(anglesLeg6, robot.leg6)


def initPositionForWalk(robot, params, constants: Constants, delay=0.0):
    # Lateral arms
    anglesLeg1 = computeIK(
        params.initLeg[0][0], params.initLeg[0][1], params.z, verbose=True
        , constants=constants)
    anglesLeg4 = computeIK(
        params.initLeg[3][0], params.initLeg[3][1], params.z, verbose=True
        , constants=constants)
    # Back and front arms
    anglesLeg2 = computeIK(
        params.initLeg[1][0], params.initLeg[1][1], params.z, verbose=True
        , constants=constants)
    anglesLeg6 = computeIK(
        params.initLeg[5][0], params.initLeg[5][1], params.z, verbose=True
        , constants=constants)
    anglesLeg5 = computeIK(
        params.initLeg[4][0], params.initLeg[4][1], params.z, verbose=True
        , constants=constants)
    anglesLeg3 = computeIK(
        params.initLeg[2][0], params.initLeg[2][1], params.z, verbose=True
        , constants=constants)

    if constants.USING_SIMPLE_ROBOT:
        setAnglesToLeg(anglesLeg1, robot.legs[1], verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg2, robot.legs[2], verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg3, robot.legs[3], verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg4, robot.legs[4], verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg5, robot.legs[5], verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg6, robot.legs[6], verbose=True)
    else:
        setAnglesToLeg(anglesLeg1, robot.leg1, verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg2, robot.leg2, verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg3, robot.leg3, verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg4, robot.leg4, verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg5, robot.leg5, verbose=True)
        time.sleep(delay)
        setAnglesToLeg(anglesLeg6, robot.leg6, verbose=True)


class Mode(Enum):
    TRIANGLE = "triangle"
    DIRECT = "direct"
    INVERSE = "inverse"


def init_controller():
    """Initialise le contrôleur PS4"""
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() > 0:
        controller = pygame.joystick.Joystick(0)
        controller.init()
        print(f"Contrôleur détecté : {controller.get_name()}")
        return controller
    else:
        print("Aucun contrôleur détecté")
        return None


def get_controller_input(controller, constants: Constants, use_rads=None):
    """Récupère les entrées du joystick
    Joystick gauche: direction et vitesse
    Joystick droit: rotation du robot
    """
    if use_rads is None:
        use_rads = constants.USE_RADS_OUTPUT
    if not controller:
        return None, None, None

    pygame.event.pump()
    left_x = controller.get_axis(0)
    left_y = controller.get_axis(1)
    right_x = controller.get_axis(2)
    right_y = controller.get_axis(3)


    deadzone = 0.15
    if abs(left_x) < deadzone and abs(left_y) < deadzone:
        direction_angle = None
    else:
        direction_angle = math.atan2(left_y, left_x)
        # if not use_rads:
        #     direction_angle = math.degrees(direction_angle)

    speed_multiplier = 1.0
    if abs(right_y) > deadzone:
        speed_multiplier = 1.0 + right_y * 10
        speed_multiplier = max(0.1, min(10, speed_multiplier))
        print(speed_multiplier)

    print(direction_angle)
    return direction_angle, speed_multiplier


def myTriangle(x: float, z: float, h: float, w: float, params, leg_id,
               robot: SimpleRobot | SimpleRobotSimulation, constants: Constants, sim: Simulation = None, leg_angle=0,
               speed=1):
    if leg_angle is None:
        return
    if leg_id % 2 == 0:
        alphas = triangle(x, z, h, w, get_time(sim) * speed, index_leg=leg_id, leg_angle=leg_angle, params=params,
                          constants=constants)
    else:
        alphas = triangle(x, z, h, w, get_time(sim) * speed + 1.0, index_leg=leg_id, leg_angle=leg_angle, params=params,
                          constants=constants)

    robot.legs[leg_id][0].goal_position = alphas[0]
    robot.legs[leg_id][1].goal_position = alphas[1]
    robot.legs[leg_id][2].goal_position = alphas[2]
    if type(robot) is SimpleRobot:
        return
    pos = computeDK(alphas[0], alphas[1], alphas[2], use_rads=True, constants=constants)
    robotPos, rpy = sim.getRobotPose()

    pos = rotaton_2D(pos[0], pos[1], pos[2], params.legAngles[leg_id - 1] + rpy[2])

    leg_center_position = rotaton_2D(constants.LEG_CENTER_POS[leg_id - 1][0],
                                     constants.LEG_CENTER_POS[leg_id - 1][1],
                                     constants.LEG_CENTER_POS[leg_id - 1][2], rpy[2])

    pos[0] += robotPos[0] + leg_center_position[0]
    pos[1] += robotPos[1] + leg_center_position[1]
    pos[2] += robotPos[2] + leg_center_position[2]
    sim.addDebugPosition(pos, duration=5)


def getAngleDirection(myEvent):
    haut = 65297 in myEvent and myEvent[65297] & p.KEY_IS_DOWN
    bas = 65298 in myEvent and myEvent[65298] & p.KEY_IS_DOWN
    gauche = 65295 in myEvent and myEvent[65295] & p.KEY_IS_DOWN
    droite = 65296 in myEvent and myEvent[65296] & p.KEY_IS_DOWN

    if haut and gauche:
        return -math.pi * 3 / 4
    if haut and droite:
        return -math.pi / 4
    if bas and gauche:
        return math.pi * 3 / 4
    if bas and droite:
        return math.pi / 4
    if haut:
        return -math.pi / 2
    if bas:
        return math.pi / 2
    if droite:
        return 0
    if gauche:
        return math.pi
    return None


def init_triangle(controls, isSim: bool):
    if not isSim:
        return None
    controls["triangle_x"] = p.addUserDebugParameter("triangle_x", 0.01, 0.8, 0)
    controls["triangle_z"] = p.addUserDebugParameter("triangle_z", -0.2, 0.3, -0.1)
    controls["triangle_h"] = p.addUserDebugParameter("triangle_h", 0.01, 0.3, 0.1)
    controls["triangle_w"] = p.addUserDebugParameter("triangle_w", 0.01, 0.3, 0.05)
    controls["speed"] = p.addUserDebugParameter("speed", 0.01, 2, 0.5)
    return controls


def init_inverse(controls, isSim: bool):
    if not isSim:
        return None
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, 0)
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, 0)
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, 0)
    return controls


def run_triangle(controls, controller, myEvent, robot, sim, params, isSim, constants: Constants):
    if isSim:
        x = p.readUserDebugParameter(controls["triangle_x"])
        z = p.readUserDebugParameter(controls["triangle_z"])
        h = p.readUserDebugParameter(controls["triangle_h"])
        w = p.readUserDebugParameter(controls["triangle_w"])
        speed = p.readUserDebugParameter(controls["speed"])
    else:
        x = 0
        z = -0.1
        h = 0.2
        w = 0.1
        speed = 1
    controller_angle, speed_multiplier = get_controller_input(controller, constants=constants)
    if controller_angle is None:
        return
    leg_angle = controller_angle if not isSim else (getAngleDirection(
        myEvent) if sim is not None else 0)

    if speed_multiplier is not None:
        speed *= speed_multiplier
        print(speed_multiplier)

    for leg_id in range(1, 7):
        myTriangle(x=x, z=z, h=h, w=w, robot=robot, params=params, leg_id=leg_id, sim=sim,
                   leg_angle=leg_angle, speed=speed, constants=constants)


def run_direct(sim, robot, params, constants: Constants):
    val = 0.05 * math.sin(get_time(sim) * 2)
    for i in range(1, 7):
        alphas = computeIKOriented(val, 0, 0, i, constants, params)
        robot.legs[i][0].goal_position = alphas[0]
        robot.legs[i][1].goal_position = alphas[1]
        robot.legs[i][2].goal_position = alphas[2]


def run_inverse(controls, sim, robot, params, constants):
    x = p.readUserDebugParameter(controls["target_x"])
    y = p.readUserDebugParameter(controls["target_y"])
    z = p.readUserDebugParameter(controls["target_z"])

    setPositionToRobot(x, y, z, robot, params, constants)
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1]) if sim is not None else None


def get_time(sim=None):
    if sim is None:
        return time.time()
    return sim.t
