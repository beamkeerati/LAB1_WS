from PathPlanning.CubicSpline import cubic_spline_planner
import matplotlib.pyplot as plt
import time
import cvxpy
import math
import numpy as np
import sys
import pathlib
from utils.angle import angle_mod

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 1.0  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.01  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 0.2  # [m]

WHEEL_RADIUS = 0.045  # [m]
WHEEL_BASE = 0.2  # [m]
TRACK_WIDTH = 0.14  # [m] track width

MAX_STEER = math.radians(10.0)  # maximum steering angle [rad]
MAX_DSTEER = math.radians(5.0)  # maximum steering speed [rad/s]
MAX_SPEED = 1.0  # maximum speed [m/s]
MIN_SPEED = -1.0  # minimum speed [m/s] - FIXED: was 1.0, should allow reverse
MAX_ACCEL = 0.5  # maximum accel [m/ss]
MAX_LINEAR_VEL = 1.0  # maximum linear velocity [m/s]
MIN_LINEAR_VEL = -1.0  # minimum linear velocity [m/s]
MAX_ANGULAR_VEL = math.radians(5)  # maximum angular velocity [rad/s]
MIN_ANGULAR_VEL = -math.radians(5)  # minimum angular velocity [rad/s]

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    return angle_mod(angle)


def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def update_state(state, a, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind : (pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind : (pind + N_IND_SEARCH)]]

    d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for ai, di, i in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC control with updating operational point iteratively
    """
    ox, oy, oyaw, ov = None, None, None, None

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)

        # FIXED: Add proper error handling for failed optimization
        if oa is None or od is None:
            print(f"MPC solver failed at iteration {i}")
            # Return previous values or zero control inputs
            if i == 0:
                oa = [0.0] * T
                od = [0.0] * T
                ox = [x0[0]] * (T + 1)
                oy = [x0[1]] * (T + 1)
                oyaw = [x0[3]] * (T + 1)
                ov = [x0[2]] * (T + 1)
            else:
                oa = poa
                od = pod
            break

        # Convert to numpy arrays if needed for arithmetic operations
        oa_array = np.array(oa)
        od_array = np.array(od)
        poa_array = np.array(poa)
        pod_array = np.array(pod)

        du = np.sum(np.abs(oa_array - poa_array)) + np.sum(np.abs(od_array - pod_array))
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)

    # FIXED: Try multiple solvers in order of preference
    solvers_to_try = [cvxpy.CLARABEL, cvxpy.OSQP, cvxpy.SCS, cvxpy.ECOS]

    solved = False
    for solver in solvers_to_try:
        try:
            if solver == cvxpy.CLARABEL:
                prob.solve(solver=solver, verbose=False, max_iter=1000)
            elif solver == cvxpy.OSQP:
                prob.solve(
                    solver=solver,
                    verbose=False,
                    max_iter=1000,
                    eps_abs=1e-4,
                    eps_rel=1e-4,
                )
            elif solver == cvxpy.SCS:
                prob.solve(solver=solver, verbose=False, max_iters=1000, eps=1e-4)
            else:
                prob.solve(solver=solver, verbose=False)

            if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
                solved = True
                break
            else:
                print(f"Solver {solver} failed with status: {prob.status}")
        except Exception as e:
            print(f"Solver {solver} encountered error: {e}")
            continue

    if solved:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])
    else:
        print("Error: Cannot solve mpc with any available solver")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = d <= GOAL_DIS

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = abs(state.v) <= STOP_SPEED

    if isgoal and isstop:
        return True

    return False


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def load_path_from_yaml(yaml_file):
    """
    Load path from YAML file (manual parsing without yaml library)

    yaml_file: path to the YAML file containing path data

    Returns:
    cx: x coordinates list
    cy: y coordinates list
    cyaw: yaw angles list
    ck: curvature list (calculated from path)
    """
    try:
        with open(yaml_file, "r") as file:
            lines = file.readlines()

        cx = []
        cy = []
        cyaw = []

        # Parse each line manually
        current_point = {}
        for line in lines:
            line = line.strip()
            if line.startswith("- x:"):
                # New point starts
                if (
                    current_point
                    and "x" in current_point
                    and "y" in current_point
                    and "yaw" in current_point
                ):
                    cx.append(current_point["x"])
                    cy.append(current_point["y"])
                    cyaw.append(current_point["yaw"])
                current_point = {}
                # Extract x value
                x_val = float(line.split("x:")[1].strip())
                current_point["x"] = x_val
            elif line.startswith("y:"):
                # Extract y value
                y_val = float(line.split("y:")[1].strip())
                current_point["y"] = y_val
            elif line.startswith("yaw:"):
                # Extract yaw value
                yaw_val = float(line.split("yaw:")[1].strip())
                current_point["yaw"] = yaw_val

        # Don't forget the last point
        if (
            current_point
            and "x" in current_point
            and "y" in current_point
            and "yaw" in current_point
        ):
            cx.append(current_point["x"])
            cy.append(current_point["y"])
            cyaw.append(current_point["yaw"])

        # Calculate curvature (simplified - set to zero for now)
        ck = [0.0] * len(cx)

        print(f"Loaded path with {len(cx)} points from {yaml_file}")

        return cx, cy, cyaw, ck

    except FileNotFoundError:
        print(f"Error: Could not find file {yaml_file}")
        return None, None, None, None
    except Exception as e:
        print(f"Error loading path from YAML: {e}")
        return None, None, None, None


def calculate_path_distance(cx, cy):
    """
    Calculate approximate path distance for dl parameter
    """
    if len(cx) < 2:
        return 1.0

    total_distance = 0.0
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        total_distance += math.sqrt(dx * dx + dy * dy)

    # Average distance between points
    dl = total_distance / (len(cx) - 1)
    return dl


def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck
