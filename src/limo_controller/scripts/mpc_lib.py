#!/usr/bin/env python3

import math
import numpy as np
import sys
import os
import cvxpy

# ROBUST IMPORT HANDLING
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# Try multiple import strategies for PathPlanning and utils
cubic_spline_planner = None
angle_mod = None

try:
    from PathPlanning.CubicSpline import cubic_spline_planner
    from utils.angle import angle_mod
except ImportError:
    try:
        from PathPlanning import cubic_spline_planner
        from utils import angle_mod
    except ImportError as e:
        print(f"Failed to import cubic_spline_planner or angle_mod: {e}")
        sys.exit(1)

show_animation = False  # Disable for ROS environment


class State:
    """Vehicle state class"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    return angle_mod(angle)


def get_linear_model_matrix(v, phi, delta, config):
    """Get linear model matrix with improved numerical conditioning"""
    A = np.zeros((config.NX, config.NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = config.DT * math.cos(phi)
    A[0, 3] = -config.DT * v * math.sin(phi)
    A[1, 2] = config.DT * math.sin(phi)
    A[1, 3] = config.DT * v * math.cos(phi)

    # Improved numerical conditioning for steering dynamics
    tan_delta = math.tan(delta)
    # Clamp to reasonable range to avoid numerical issues
    tan_delta = max(-10.0, min(10.0, tan_delta))
    A[3, 2] = config.DT * tan_delta / config.WB

    B = np.zeros((config.NX, config.NU))
    B[2, 0] = config.DT

    # Improved numerical conditioning for steering input
    cos_delta_sq = math.cos(delta) ** 2
    cos_delta_sq = max(0.01, cos_delta_sq)  # Avoid division by zero
    B[3, 1] = config.DT * v / (config.WB * cos_delta_sq)

    C = np.zeros(config.NX)
    C[0] = config.DT * v * math.sin(phi) * phi
    C[1] = -config.DT * v * math.cos(phi) * phi
    C[3] = -config.DT * v * delta / (config.WB * cos_delta_sq)

    return A, B, C


def update_state(state, a, delta, config):
    """Update state with improved constraint handling"""
    # Input constraints with safety margins
    if delta >= config.MAX_STEER:
        delta = config.MAX_STEER * 0.95  # Small safety margin
    elif delta <= -config.MAX_STEER:
        delta = -config.MAX_STEER * 0.95

    state.x = state.x + state.v * math.cos(state.yaw) * config.DT
    state.y = state.y + state.v * math.sin(state.yaw) * config.DT
    state.yaw = state.yaw + state.v / config.WB * math.tan(delta) * config.DT
    state.v = state.v + a * config.DT

    # Apply speed limits with safety margins
    if state.v > config.MAX_SPEED:
        state.v = config.MAX_SPEED * 0.95
    elif state.v < config.MIN_SPEED:
        state.v = config.MIN_SPEED * 0.95

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind, config):
    """Calculate nearest index with improved search"""
    search_range = min(config.N_IND_SEARCH, len(cx) - pind)
    if search_range <= 0:
        return pind, 0.0

    dx = [state.x - icx for icx in cx[pind : (pind + search_range)]]
    dy = [state.y - icy for icy in cy[pind : (pind + search_range)]]

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


def predict_motion(x0, oa, od, xref, config):
    """Predict motion with improved state validation"""
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for ai, di, i in zip(oa, od, range(1, config.T + 1)):
        state = update_state(state, ai, di, config)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od, config):
    """MPC control with improved error handling and feasibility"""
    ox, oy, oyaw, ov = None, None, None, None

    if oa is None or od is None:
        oa = [0.0] * config.T
        od = [0.0] * config.T

    for i in range(config.MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref, config)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref, config)

        if oa is None or od is None:
            print(f"MPC solver failed at iteration {i}")
            if i == 0:
                # Conservative fallback
                oa = [0.0] * config.T
                od = [0.0] * config.T
                ox = [x0[0]] * (config.T + 1)
                oy = [x0[1]] * (config.T + 1)
                oyaw = [x0[3]] * (config.T + 1)
                ov = [x0[2]] * (config.T + 1)
            else:
                oa = poa
                od = pod
            break

        # Check convergence
        oa_array = np.array(oa)
        od_array = np.array(od)
        poa_array = np.array(poa)
        pod_array = np.array(pod)

        du = np.sum(np.abs(oa_array - poa_array)) + np.sum(np.abs(od_array - pod_array))
        if du <= config.DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref, config):
    """Linear MPC control optimized for robot with 10-degree steering limit"""
    try:
        x = cvxpy.Variable((config.NX, config.T + 1))
        u = cvxpy.Variable((config.NU, config.T))

        cost = 0.0
        constraints = []

        for t in range(config.T):
            cost += cvxpy.quad_form(u[:, t], config.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], config.Q)

            A, B, C = get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t], config
            )
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            # Relaxed steering rate constraint
            if t < (config.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], config.Rd)
                # More lenient steering rate constraint
                max_dsteer_dt = config.MAX_DSTEER * config.DT * 2.0  # Double the limit
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= max_dsteer_dt]

        cost += cvxpy.quad_form(xref[:, config.T] - x[:, config.T], config.Qf)

        # Initial state constraint
        constraints += [x[:, 0] == x0]

        # Relaxed speed constraints with safety margins
        speed_margin = 0.1
        constraints += [x[2, :] <= config.MAX_SPEED - speed_margin]
        constraints += [x[2, :] >= config.MIN_SPEED + speed_margin]

        # Conservative control constraints for 10-degree steering limit
        accel_margin = 0.05
        steer_margin = 0.02  # Smaller margin for limited steering range
        constraints += [cvxpy.abs(u[0, :]) <= config.MAX_ACCEL - accel_margin]
        constraints += [cvxpy.abs(u[1, :]) <= config.MAX_STEER - steer_margin]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)

        # Try multiple solvers with improved settings
        solvers_to_try = [
            (cvxpy.CLARABEL, {"verbose": False, "max_iter": 2000, "tol_feas": 1e-6}),
            (
                cvxpy.OSQP,
                {"verbose": False, "max_iter": 2000, "eps_abs": 1e-5, "eps_rel": 1e-5},
            ),
            (cvxpy.SCS, {"verbose": False, "max_iters": 2000, "eps": 1e-5}),
            (cvxpy.ECOS, {"verbose": False, "max_iters": 2000}),
        ]

        solved = False
        for solver, solver_opts in solvers_to_try:
            try:
                prob.solve(solver=solver, **solver_opts)

                if (
                    prob.status == cvxpy.OPTIMAL
                    or prob.status == cvxpy.OPTIMAL_INACCURATE
                ):
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

    except Exception as e:
        print(f"MPC optimization error: {e}")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind, config):
    """Calculate reference trajectory with improved handling"""
    xref = np.zeros((config.NX, config.T + 1))
    dref = np.zeros((1, config.T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind, config)

    if pind >= ind:
        ind = pind

    # Ensure index is within bounds
    ind = max(0, min(ind, ncourse - 1))

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0

    travel = 0.0

    for i in range(config.T + 1):
        travel += abs(state.v) * config.DT
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


def check_goal(state, goal, tind, nind, config):
    """Check if goal is reached"""
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = d <= config.GOAL_DIS

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = abs(state.v) <= config.STOP_SPEED

    if isgoal and isstop:
        return True

    return False


def calc_speed_profile(cx, cy, cyaw, target_speed, config):
    """Calculate speed profile with direction handling"""
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

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


def smooth_yaw(yaw, config):
    """Smooth yaw angles to prevent discontinuities"""
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def calculate_path_distance(cx, cy):
    """Calculate approximate path distance for dl parameter"""
    if len(cx) < 2:
        return 1.0

    total_distance = 0.0
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        total_distance += math.sqrt(dx * dx + dy * dy)

    # Average distance between points
    dl = total_distance / (len(cx) - 1)
    return max(dl, 0.1)  # Ensure minimum distance


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
