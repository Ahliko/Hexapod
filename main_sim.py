import time
import pygame

import utils
import pybullet as p
from onshape_to_robot.simulation import Simulation
import argparse
import pypot.dynamixel

from constants import Constants

STOP = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        "-m",
        type=str,
        default="simulation",
        help="Available modes : simulation, robot",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=str,
        default="/dev/ttyACM0",
        help="Port for the robot",
    )
    robot: utils.SimpleRobotSimulation | utils.SimpleRobot = None
    constants = Constants()
    try:
        args = parser.parse_args()
        if args.mode == "simulation":
            robotPath = "phantomx_description/urdf/phantomx.urdf"
            sim = Simulation(robotPath=robotPath, gui=True, panels=True, useUrdfInertia=False)
            constants.ROBOT_TYPE = constants.PHANTOMX_SIMULATION
            constants.update()
            robot: utils.SimpleRobotSimulation = utils.SimpleRobotSimulation(sim, constants)
            params = utils.Parameters(
                constants=constants,
                freq=50,
                speed=1,
                z=-0.05,
                travelDistancePerStep=0.8,
                lateralDistance=0.2,
                frontDistance=0.8,
                frontStart=0.3,
                method="minJerk",
            )


        else:
            dxl_io = pypot.dynamixel.DxlIO(args.port, baudrate=1000000)
            sim = None
            constants.ROBOT_TYPE = constants.PHANTOMX
            constants.update()
            robot: utils.SimpleRobot = utils.SimpleRobot(dxl_io, constants)
            params = utils.Parameters(
                constants=constants,
                freq=50,
                speed=1,
                z=80,
                travelDistancePerStep=80,
                lateralDistance=110,
                frontDistance=87,
                frontStart=32,
                method="minJerk",
            )
        robot.init(constants=constants)
        mode_actuel = utils.Mode.DIRECT
        robot.drawOn = False
        robot.disable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])

        robot.tick_read(constants)
        # print(utils.computeDK(robot.legs[1][0].present_position, robot.legs[1][1].present_position, robot.legs[1][2].present_position, constants))
        # print(kinematics2.computeDK(robot.legs[1][0].present_position, robot.legs[1][1].present_position, robot.legs[1][2].present_position))

        # print(utils.computeIK(0.2, 0, 0, constants))
        # print(kinematics2.computeIK(0.2, 0, 0))

        # print(utils.computeIKOriented(0.2, 0, 0, 1, constants, params))
        # print(kinematics2.computeIKOriented(0.2, 0, 0, 1, params=params))

        # while True:
        #     continue
        for i in range(1, 7):
            alphas = utils.computeIKOriented(0, 0, 0, i, constants, params)
            robot.legs[i][0].goal_position = alphas[0]
            robot.legs[i][1].goal_position = alphas[1]
            robot.legs[i][2].goal_position = alphas[2]
        print("enabling torque")
        robot.enable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
        print("enabled torque")
        robot.smooth_tick_read_and_write(delay=2, constants=constants)
        # while True:
        #     continue

        pygame.init()
        controller = utils.init_controller()
        controls = {}
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1]) if sim is not None else None

        while True:
            if STOP or controller.get_button(9):
                robot.disable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
                pygame.quit()
                return
            my_event = p.getKeyboardEvents() if sim is not None else None
            pygame.event.pump()
            if controller.get_button(1):
                robot.disable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
                continue
            if (my_event is not None and -1 in my_event and my_event[-1] & p.KEY_WAS_RELEASED) or controller.get_button(
                    0) if controller is not None else False:
                if controller.get_button(0) if controller is not None else False:
                    time.sleep(0.1)
                    pygame.event.pump()
                    if controller.get_button(0):
                        continue
                if sim is not None:
                    p.removeAllUserParameters()
                    p.removeAllUserDebugItems()
                if mode_actuel == utils.Mode.INVERSE:
                    mode_actuel = mode_actuel.DIRECT
                elif mode_actuel == utils.Mode.TRIANGLE:
                    mode_actuel = utils.Mode.INVERSE
                    controls = utils.init_inverse(controls, sim is not None)
                else:
                    mode_actuel = utils.Mode.TRIANGLE
                    controls = utils.init_triangle(controls, sim is not None)
            if mode_actuel == utils.Mode.INVERSE:
                utils.run_inverse(controls, sim, robot, params, constants=constants) if sim is not None else None
            elif mode_actuel == utils.Mode.TRIANGLE:
                if controller.get_button(7):
                    utils.run_triangle(controls, controller, my_event, robot, sim, params, sim is not None,
                                       constants=constants)
                else:
                    utils.get_controller_input(controller, constants=constants)
            elif mode_actuel == utils.Mode.DIRECT:
                utils.run_direct(sim, robot, params, constants)

            robot.tick_read_and_write(constants=constants)
            # print(robot)

            robot.tickSim() if sim is not None else None
            pygame.event.pump()
    finally:
        if robot is not None:
            robot.disable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
        pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        STOP = True
