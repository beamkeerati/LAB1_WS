#!/usr/bin/env python3

import math
import numpy as np
import sys
import os
import casadi as ca

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


def calc_nearest_index(state, cx, cy, cyaw, pind, config):
    """Calculate nearest index with improved search and yaw consideration"""
    search_range = min(config.N_IND_SEARCH, len(cx) - pind)
    if search_range <= 0:
        return pind, 0.0

    # Expand search range for better path following
    search_start = max(0, pind - 5)  # Look backward too
    search_end = min(len(cx), pind + search_range + 10)  # Look further ahead

    dx = [state.x - icx for icx in cx[search_start:search_end]]
    dy = [state.y - icy for icy in cy[search_start:search_end]]

    d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind) + search_start

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    # DEBUG: Log nearest index calculation (reduce frequency)
    if np.random.random() < 0.1:  # Only 10% of the time
        print(
            f"Nearest index search: current_ind={pind}, found_ind={ind}, distance={mind:.3f}"
        )
        print(f"  Robot: ({state.x:.3f}, {state.y:.3f}, yaw={state.yaw:.3f})")
        print(f"  Target: ({cx[ind]:.3f}, {cy[ind]:.3f}, yaw={cyaw[ind]:.3f})")

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
    """MPC control with improved error handling and feasibility using CasADi"""
    ox, oy, oyaw, ov = None, None, None, None

    if oa is None or od is None:
        oa = [0.0] * config.T
        od = [0.0] * config.T

    for i in range(config.MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref, config)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control_casadi(
            xref, xbar, x0, dref, config
        )

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


def linear_mpc_control_casadi(xref, xbar, x0, dref, config):
    """Linear MPC control using CasADi with IPOPT solver"""
    try:
        # DEBUG: Check input values for sanity (reduce frequency)
        if np.random.random() < 0.1:  # Only 10% of the time
            print(f"=== MPC OPTIMIZATION DEBUG (CasADi) ===")
            print(f"x0: [{x0[0]:.3f}, {x0[1]:.3f}, {x0[2]:.3f}, {x0[3]:.3f}]")
            print(
                f"xref[0]: [{xref[0,0]:.3f}, {xref[1,0]:.3f}, {xref[2,0]:.3f}, {xref[3,0]:.3f}]"
            )
            print(
                f"Speed range in ref: {np.min(xref[2,:]):.3f} to {np.max(xref[2,:]):.3f}"
            )
            print(
                f"Constraints: v in [{config.MIN_SPEED:.3f}, {config.MAX_SPEED:.3f}], steer in [{-config.MAX_STEER:.3f}, {config.MAX_STEER:.3f}]"
            )

        # Decision variables
        x = ca.MX.sym("x", config.NX, config.T + 1)  # States over horizon
        u = ca.MX.sym("u", config.NU, config.T)  # Controls over horizon

        # Parameters (reference trajectory)
        x_ref = ca.MX.sym("x_ref", config.NX, config.T + 1)
        x0_param = ca.MX.sym("x0", config.NX)
        xbar_param = ca.MX.sym("xbar", config.NX, config.T + 1)
        dref_param = ca.MX.sym("dref", 1, config.T + 1)

        # Cost function
        cost = 0
        constraints = []
        lbg = []  # Lower bounds for constraints
        ubg = []  # Upper bounds for constraints

        # Initial state constraint
        constraints.append(x[:, 0] - x0_param)
        lbg.extend([0.0] * config.NX)
        ubg.extend([0.0] * config.NX)

        # Build cost and dynamics constraints
        for t in range(config.T):
            # Control cost
            cost += ca.mtimes([u[:, t].T, config.R, u[:, t]])

            # State cost (except initial state)
            if t > 0:
                state_error = x_ref[:, t] - x[:, t]
                cost += ca.mtimes([state_error.T, config.Q, state_error])

            # Get linearized dynamics matrices
            A, B, C = get_linear_model_matrix(
                float(xbar[2, t]), float(xbar[3, t]), float(dref[0, t]), config
            )

            # Convert to CasADi format
            A_ca = ca.DM(A)
            B_ca = ca.DM(B)
            C_ca = ca.DM(C)

            # Dynamics constraint: x_{t+1} = A*x_t + B*u_t + C
            dynamics = x[:, t + 1] - (
                ca.mtimes(A_ca, x[:, t]) + ca.mtimes(B_ca, u[:, t]) + C_ca
            )
            constraints.append(dynamics)
            lbg.extend([0.0] * config.NX)
            ubg.extend([0.0] * config.NX)

            # Control rate constraints
            if t < (config.T - 1):
                rate_cost = ca.mtimes(
                    [(u[:, t + 1] - u[:, t]).T, config.Rd, (u[:, t + 1] - u[:, t])]
                )
                cost += rate_cost

                # Steering rate constraint
                max_dsteer_dt = config.MAX_DSTEER * config.DT * 2.0
                constraints.append(u[1, t + 1] - u[1, t])
                lbg.append(-max_dsteer_dt)
                ubg.append(max_dsteer_dt)

        # Terminal cost
        terminal_error = x_ref[:, config.T] - x[:, config.T]
        cost += ca.mtimes([terminal_error.T, config.Qf, terminal_error])

        # State and control bounds
        lbx = []  # Lower bounds for decision variables
        ubx = []  # Upper bounds for decision variables

        # State bounds: [x, y, v, yaw]
        for t in range(config.T + 1):
            lbx.extend(
                [-ca.inf, -ca.inf, config.MIN_SPEED + 0.1, -ca.inf]
            )  # x, y, v, yaw
            ubx.extend([ca.inf, ca.inf, config.MAX_SPEED - 0.1, ca.inf])

        # Control bounds: [acceleration, steering]
        for t in range(config.T):
            lbx.extend([-config.MAX_ACCEL + 0.05, -config.MAX_STEER + 0.02])
            ubx.extend([config.MAX_ACCEL - 0.05, config.MAX_STEER - 0.02])

        # Create optimization variables vector
        opt_vars = ca.vertcat(ca.reshape(x, -1, 1), ca.reshape(u, -1, 1))

        # Create parameter vector
        params = ca.vertcat(
            ca.reshape(x_ref, -1, 1),
            x0_param,
            ca.reshape(xbar_param, -1, 1),
            ca.reshape(dref_param, -1, 1),
        )

        # Create the NLP
        nlp = {"x": opt_vars, "f": cost, "g": ca.vertcat(*constraints), "p": params}

        # Solver options
        opts = {
            "ipopt.print_level": 0,
            "ipopt.max_iter": 100,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-3,
            "print_time": False,
            "verbose": False,
        }

        # Create solver
        solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        # Prepare parameter values
        x_ref_flat = xref.flatten(order="F")
        xbar_flat = xbar.flatten(order="F")
        dref_flat = dref.flatten(order="F")
        param_values = np.concatenate([x_ref_flat, x0, xbar_flat, dref_flat])

        # Initial guess (warm start with previous solution if available)
        x0_guess = np.tile(x0, config.T + 1)  # Repeat initial state
        u0_guess = np.zeros(config.NU * config.T)  # Zero controls
        init_guess = np.concatenate([x0_guess, u0_guess])

        # Solve
        solution = solver(
            x0=init_guess, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=param_values
        )

        # Check if solver succeeded
        if solver.stats()["success"]:
            # Extract solution
            opt_solution = solution["x"].full().flatten()

            # Extract states and controls
            x_opt = opt_solution[: config.NX * (config.T + 1)].reshape(
                (config.NX, config.T + 1), order="F"
            )
            u_opt = opt_solution[config.NX * (config.T + 1) :].reshape(
                (config.NU, config.T), order="F"
            )

            ox = x_opt[0, :].tolist()
            oy = x_opt[1, :].tolist()
            ov = x_opt[2, :].tolist()
            oyaw = x_opt[3, :].tolist()
            oa = u_opt[0, :].tolist()
            odelta = u_opt[1, :].tolist()

            # DEBUG: Check solution sanity
            if np.random.random() < 0.1:  # Only 10% of the time
                print(f"✓ CasADi solver succeeded")
                print(f"  Acceleration range: {np.min(oa):.3f} to {np.max(oa):.3f}")
                print(
                    f"  Steering range: {np.min(odelta):.3f} to {np.max(odelta):.3f} ({np.degrees(np.min(odelta)):.1f} to {np.degrees(np.max(odelta)):.1f} deg)"
                )
                print(f"  Speed range: {np.min(ov):.3f} to {np.max(ov):.3f}")
                print(f"  Solver iterations: {solver.stats()['iter_count']}")
                print(f"  Solve time: {solver.stats()['t_wall_total']:.4f}s")

        else:
            print(f"✗ CasADi solver failed: {solver.stats()['return_status']}")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    except Exception as e:
        print(f"CasADi MPC optimization error: {e}")
        import traceback

        traceback.print_exc()
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind, config):
    """Calculate reference trajectory with improved handling and yaw wrapping"""
    xref = np.zeros((config.NX, config.T + 1))
    dref = np.zeros((1, config.T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind, config)

    # Ensure we're progressing forward along the path
    if pind >= ind:
        ind = pind

    # Ensure index is within bounds
    ind = max(0, min(ind, ncourse - 1))

    # Handle yaw wrapping for reference
    ref_yaw = cyaw[ind]
    # Wrap reference yaw to be close to current robot yaw
    while ref_yaw - state.yaw > math.pi:
        ref_yaw -= 2 * math.pi
    while ref_yaw - state.yaw < -math.pi:
        ref_yaw += 2 * math.pi

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    # IMPORTANT: Clamp reference speed to respect constraints
    ref_speed = max(config.MIN_SPEED + 0.1, min(config.MAX_SPEED - 0.1, sp[ind]))
    xref[2, 0] = ref_speed
    xref[3, 0] = ref_yaw
    dref[0, 0] = 0.0

    travel = 0.0
    prev_yaw = ref_yaw

    for i in range(1, config.T + 1):
        travel += abs(state.v) * config.DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            # IMPORTANT: Clamp reference speed to respect constraints
            ref_speed = max(
                config.MIN_SPEED + 0.1, min(config.MAX_SPEED - 0.1, sp[ind + dind])
            )
            xref[2, i] = ref_speed

            # Handle yaw continuity in reference trajectory
            next_yaw = cyaw[ind + dind]
            while next_yaw - prev_yaw > math.pi:
                next_yaw -= 2 * math.pi
            while next_yaw - prev_yaw < -math.pi:
                next_yaw += 2 * math.pi
            xref[3, i] = next_yaw
            prev_yaw = next_yaw

            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            # IMPORTANT: Clamp reference speed to respect constraints
            ref_speed = max(
                config.MIN_SPEED + 0.1, min(config.MAX_SPEED - 0.1, sp[ncourse - 1])
            )
            xref[2, i] = ref_speed
            xref[3, i] = prev_yaw  # Keep consistent yaw at end
            dref[0, i] = 0.0

    # DEBUG: Verify reference trajectory is within constraints
    ref_speeds = xref[2, :]
    ref_yaws = xref[3, :]
    yaw_changes = np.diff(ref_yaws)
    max_yaw_change = np.max(np.abs(yaw_changes)) if len(yaw_changes) > 0 else 0

    print(f"=== REFERENCE TRAJECTORY DEBUG ===")
    print(
        f"Robot state: pos=({state.x:.3f}, {state.y:.3f}), yaw={state.yaw:.3f}, v={state.v:.3f}"
    )
    print(
        f"Ref[0]: pos=({xref[0,0]:.3f}, {xref[1,0]:.3f}), yaw={xref[3,0]:.3f}, v={xref[2,0]:.3f}"
    )
    print(
        f"Yaw error: {abs(state.yaw - xref[3,0]):.3f} rad ({math.degrees(abs(state.yaw - xref[3,0])):.1f} deg)"
    )
    print(
        f"Position error: {math.sqrt((state.x - xref[0,0])**2 + (state.y - xref[1,0])**2):.3f} m"
    )

    if max_yaw_change > math.pi / 4:
        print(
            f"WARNING: Large yaw change in horizon: {max_yaw_change:.3f} rad ({math.degrees(max_yaw_change):.1f} deg)"
        )

    if not all(config.MIN_SPEED <= s <= config.MAX_SPEED for s in ref_speeds):
        print(f"WARNING: Reference trajectory still has constraint violations!")
        print(f"Speed range: {np.min(ref_speeds):.3f} to {np.max(ref_speeds):.3f}")
        print(f"Constraints: {config.MIN_SPEED:.3f} to {config.MAX_SPEED:.3f}")

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
    """Calculate speed profile with direction handling and constraint compliance"""
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

    # DEBUG: Log speed profile calculation
    print(f"=== SPEED PROFILE DEBUG ===")
    print(f"Target speed: {target_speed}")
    print(f"Path length: {len(cx)}")
    print(f"Speed constraints: {config.MIN_SPEED:.3f} to {config.MAX_SPEED:.3f}")

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

    # IMPORTANT: Clamp speeds to respect constraints
    for i in range(len(speed_profile)):
        speed_profile[i] = max(
            config.MIN_SPEED, min(config.MAX_SPEED, speed_profile[i])
        )

    # DEBUG: Log speed profile statistics
    forward_count = sum(1 for s in speed_profile if s > 0)
    reverse_count = sum(1 for s in speed_profile if s < 0)
    print(f"Speed profile: {forward_count} forward, {reverse_count} reverse points")
    print(f"Speed range: {min(speed_profile):.3f} to {max(speed_profile):.3f}")
    print(
        f"All speeds within constraints: {all(config.MIN_SPEED <= s <= config.MAX_SPEED for s in speed_profile)}"
    )

    return speed_profile


def smooth_yaw(yaw, config):
    """Smooth yaw angles to prevent discontinuities with improved handling"""
    print(f"=== YAW SMOOTHING DEBUG ===")
    print(f"Original yaw range: {min(yaw):.3f} to {max(yaw):.3f}")

    smoothed_yaw = yaw.copy()
    large_jumps = 0

    for i in range(len(smoothed_yaw) - 1):
        dyaw = smoothed_yaw[i + 1] - smoothed_yaw[i]
        original_dyaw = dyaw

        while dyaw >= math.pi:
            smoothed_yaw[i + 1] -= math.pi * 2.0
            dyaw = smoothed_yaw[i + 1] - smoothed_yaw[i]
            large_jumps += 1

        while dyaw <= -math.pi:
            smoothed_yaw[i + 1] += math.pi * 2.0
            dyaw = smoothed_yaw[i + 1] - smoothed_yaw[i]
            large_jumps += 1

        if abs(original_dyaw) > math.pi / 2:
            print(f"  Large yaw jump at index {i}: {original_dyaw:.3f} -> {dyaw:.3f}")

    print(f"Smoothed yaw range: {min(smoothed_yaw):.3f} to {max(smoothed_yaw):.3f}")
    print(f"Fixed {large_jumps} large yaw jumps")

    return smoothed_yaw


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
