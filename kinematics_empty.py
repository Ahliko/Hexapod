import math
from turtledemo.paint import switchupdown
from zlib import Z_RLE
from constants import Constants


def alkashi(a, b, c, sign=1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


def set_default_parameters(l1: float, l2: float, l3: float, use_rads: bool, use_mm: bool, sign: int,
                           constants: Constants):
    if not isinstance(constants, Constants):
        raise TypeError("L'argument 'constants' doit être un objet de type Constants")
    l1 = constants.constL1 if l1 is None else l1
    l2 = constants.constL2 if l2 is None else l2
    l3 = constants.constL3 if l3 is None else l3
    use_rads = constants.USE_RADS_INPUT if use_rads is None else use_rads
    use_mm = constants.USE_MM_OUTPUT if use_mm is None else use_mm
    sign = constants.DEFAULT_COMPUTE_IK_SIGN if sign is None else sign
    return l1, l2, l3, use_rads, use_mm, sign


def computeDK(
        theta1: float,
        theta2: float,
        theta3: float,
        constants: Constants,
        l1: float = None,
        l2: float = None,
        l3: float = None,
        use_rads: bool = None,
        use_mm: bool = None,
):
    l1, l2, l3, use_rads, use_mm, _ = set_default_parameters(l1=l1, l2=l2, l3=l3, use_rads=use_rads, use_mm=use_mm,
                                                             sign=-1, constants=constants)
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = constants.THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (constants.THETA2_MOTOR_SIGN * theta2 - constants.theta2Correction) * angle_unit
    theta3 = (constants.THETA3_MOTOR_SIGN * theta3 - constants.theta3Correction) * angle_unit

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def computeDKDetailed(
        theta1,
        theta2,
        theta3,
        constants: Constants,
        l1: float = None,
        l2: float = None,
        l3: float = None,
        use_rads: bool = None,
        use_mm: bool = None,
):
    l1, l2, l3, use_rads, use_mm, _ = set_default_parameters(l1=l1, l2=l2, l3=l3, use_rads=use_rads, use_mm=use_mm,
                                                             sign=-1, constants=constants)
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = constants.THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (constants.THETA2_MOTOR_SIGN * theta2 - constants.theta2Correction) * angle_unit
    theta3 = (constants.THETA3_MOTOR_SIGN * theta3 - constants.theta3Correction) * angle_unit

    print(
        "corrected angles={}, {}, {}".format(
            theta1 * (1.0 / angle_unit),
            theta2 * (1.0 / angle_unit),
            theta3 * (1.0 / angle_unit),
        )
    )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = computeDK(
        theta1=theta1_verif, theta2=theta2_verif, theta3=theta3_verif, l1=l1, l2=l2, l3=l3, use_rads=use_rads,
        use_mm=use_mm, constants=constants
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


def computeIK(
        x,
        y,
        z,
        constants: Constants,
        l1=None,
        l2=None,
        l3=None,
        verbose=False,
        use_rads=None,
        sign=None,
        use_mm=None,
):
    l1, l2, l3, use_rads, use_mm, sign = set_default_parameters(l1=l1, l2=l2, l3=l3, use_rads=use_rads, use_mm=use_mm,
                                                                sign=sign, constants=constants)

    # print(f"l1 : {l1}, l2 : {l2}, l3 : {l3}, use_rads : {use_rads}, use_mm : {use_mm}")
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    if y == 0 and x == 0:
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)
    xp = math.sqrt(x * x + y * y) - l1
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    theta2 = alkashi(l2, d, l3, sign=sign) - constants.Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alkashi(l2, l3, d, sign=sign)
    # print(f"Z_DIRECTION : {constants.Z_DIRECTION}, d : {d}, xp : {xp}, sign : {sign}")

    if use_rads:

        result = [
            angleRestrict(constants.THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                constants.THETA2_MOTOR_SIGN * (theta2 + constants.theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                constants.THETA3_MOTOR_SIGN * (theta3 + constants.theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        # print(f"THETA1_MOTOR_SIGN : {constants.THETA1_MOTOR_SIGN}, theta2 : {theta2}, theta3 : {theta3}")
        result = [
            angleRestrict(constants.THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                constants.THETA2_MOTOR_SIGN * (math.degrees(theta2) + constants.theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                constants.THETA3_MOTOR_SIGN * (math.degrees(theta3) + constants.theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x, y, z, result[0], result[1], result[2],
            )
        )
    # print(f"result : {result}")
    return result


def computeIKOriented(x: float, y: float, z: float, leg_id: int, constants: Constants, params,
                      extra_theta: float = 0):
    """
    Oriented IK function for the leg leg_id of the robot.
    Takes x, y, z coordinates of the leg's frame (centered of the first motor).
    Rotates this to robot's frame.
    And then calls the ComputeIK function with the x, y, z coordinates rotated plus a coordinate offset (expressed in robot's frame).
    For example : Calling this function with x = 0, y = 0, z = 0 returns the default position of the leg leg_id.
    For example : Calling this function with x = 0.1, y = 0, z = 0 on all legs will move backwards the body of the hexapod by 10cm.
    :param x:
    :param y:
    :param z:
    :param leg_id:
    :param constants:
    :param params:
    :param extra_theta:
    :return: (theta1, theta2, theta3)
    """
    if leg_id < 1 or leg_id > 6:
        raise RuntimeError("ERROR unknown legID '{}'".format(leg_id))
    if not isinstance(constants, Constants):
        raise TypeError("L'argument 'constants' doit être un objet de type Constants")

    theta = params.legAngles[leg_id - 1]

    theta = theta + extra_theta
    # Applying a rotation
    new_x, new_y, _ = rotaton_2D(x, y, z, -theta)
    # print(f"new_x : {new_x}, new_y : {new_y}, theta : {theta}")
    # print(
    #     f"x : {new_x + params.initLeg[leg_id - 1][0]}, y : {new_y + params.initLeg[leg_id - 1][1]}, z : {z + params.z}")
    if constants.USE_MM_OUTPUT:
        new_x = new_x * 250
        new_y = new_y * 250
        z = z * 250

    return computeIK(
        x=new_x + params.initLeg[leg_id - 1][0],
        y=new_y + params.initLeg[leg_id - 1][1],
        z=z + params.z,
        constants=constants,
    )
    # pos = rotaton_2D(x, y, z, -constants.LEG_ANGLES[leg_id - 1] - extra_theta)
    # return computeIK(constants.LEG_END_POS[leg_id - 1][0] + pos[0], constants.LEG_END_POS[leg_id - 1][1] + pos[1],
    #                  constants.LEG_END_POS[leg_id - 1][2] + pos[2], constants=constants)


def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


def rotaton_2D(x, y, z, theta):
    x_prime = x * math.cos(theta) - y * math.sin(theta)
    y_prime = x * math.sin(theta) + y * math.cos(theta)

    return [x_prime, y_prime, z]


def circle(x, z, rayon, t, duration):
    y = rayon * math.sin(t / duration * 2 * math.pi)
    z = z + rayon * math.cos(t / duration * 2 * math.pi)
    return computeIK(x, y, z)


def segmentByPercent(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, percent,
                     constants: Constants,
                     index_leg: int = None, leg_angle: float = 0, params=None):
    x = segment_x1 + (segment_x2 - segment_x1) * percent
    y = segment_y1 + (segment_y2 - segment_y1) * percent
    z = segment_z1 + (segment_z2 - segment_z1) * percent
    if index_leg is not None:
        return computeIKOriented(x, y, z, leg_id=index_leg, extra_theta=leg_angle, params=params, constants=constants)
    return computeIK(x, y, z)


def segment(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, t, duration, constants: Constants):
    percent = t % duration / duration if t % (2 * duration) < duration else 1 - t % duration / duration
    return segmentByPercent(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, percent,
                            constants=constants)


def triangle(x, z, h, w, t, constants: Constants, index_leg: int = None, leg_angle: float = 0, params=None):
    point1 = (x, 0, z)
    point2 = (x, w, z)
    point3 = (x, w / 2, z + h)

    duration1 = 1.0
    duration2 = 0.5
    duration3 = 0.5
    total_duration = duration1 + duration2 + duration3

    t_cycle = t % total_duration

    if t_cycle < duration1:
        percent = t_cycle / duration1
        return segmentByPercent(point1[0], point1[1], point1[2], point2[0], point2[1], point2[2], percent,
                                index_leg=index_leg, leg_angle=leg_angle, params=params, constants=constants)
    elif t_cycle < duration1 + duration2:
        percent = (t_cycle - duration1) / duration2
        return segmentByPercent(point2[0], point2[1], point2[2], point3[0], point3[1], point3[2], percent,
                                index_leg=index_leg, leg_angle=leg_angle, params=params, constants=constants)
    percent = (t_cycle - duration1 - duration2) / duration3
    return segmentByPercent(point3[0], point3[1], point3[2], point1[0], point1[1], point1[2], percent,
                            index_leg=index_leg, leg_angle=leg_angle, params=params, constants=constants)
