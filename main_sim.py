import time
import pygame
import utils
import pybullet as p
from onshape_to_robot.simulation import Simulation
import argparse
import pypot.dynamixel
from constants import Constants

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        "-m",
        type=str,
        default="simulation",
        help="Available modes : simulation, robot",
    )
    robot: utils.SimpleRobotSimulation | utils.SimpleRobot
    constants = Constants()
    try:
        args = parser.parse_args()
        if args.mode == "simulation":
            robotPath = "phantomx_description/urdf/phantomx.urdf"
            sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
            constants.ROBOT_TYPE = "PHANTOMX"
            constants.update()
            robot: utils.SimpleRobotSimulation = utils.SimpleRobotSimulation(sim, constants)


        else:
            dxl_io = pypot.dynamixel.DxlIO('/dev/ttyACM0', baudrate=1000000)
            sim = None
            constants.ROBOT_TYPE="PHANTOMX"
            constants.update()
            robot : utils.SimpleRobot = utils.SimpleRobot(dxl_io, constants)
        robot.init()
        # while True:
        #     robot.tick_read()
        #     for m in robot.motors():
        #         print(m)
        mode_actuel = utils.Mode.DIRECT
        params = utils.Parameters(constants)
        robot.drawOn = False
        robot.disable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
        for i in range(1, 7):
            alphas = utils.computeIKOriented(0, 0, 0, i, constants, params)
            robot.legs[i][0].goal_position = alphas[0]
            robot.legs[i][1].goal_position = alphas[1]
            robot.legs[i][2].goal_position = alphas[2]
        print("enabling torque")
        robot.enable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
        print("enabled torque")
        robot.smooth_tick_read_and_write(delay=2, constants=constants, verbose=True)
        while True:
            continue

        pygame.init()
        controller = utils.init_controller()
        controls = {}
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1]) if sim is not None else None

        while True:
            my_event = p.getKeyboardEvents() if sim is not None else None
            pygame.event.pump()
            if (my_event is not None and -1 in my_event and my_event[-1] & p.KEY_WAS_RELEASED) or controller.get_button(0) if controller is not None else False:
                if controller.get_button(0) if controller is not None else False:
                    time.sleep(0.1)
                    pygame.event.pump()
                    if controller.get_button(0):
                        continue
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
                utils.run_inverse(controls, sim, robot, params, constants=constants)
            elif mode_actuel == utils.Mode.TRIANGLE:
                utils.run_triangle(controls, controller, my_event, robot, sim, params, sim is not None, constants=constants)
            elif mode_actuel == utils.Mode.DIRECT:
                utils.run_direct(sim, robot, params)

            robot.tick_read_and_write(constants=constants)
            robot.tickSim() if sim is not None else None
            pygame.event.pump()
    finally:
        if robot is not None:
            robot.disable_torque([robot.motors()[i].id for i in range(len(robot.motors()))])
        pygame.quit()


if __name__ == "__main__":
    main()
