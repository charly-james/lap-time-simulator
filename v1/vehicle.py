"""Point-mass lap time simulator (v1): vehicle parameters and speed profile solver."""

import numpy as np

# Known variables:
mass = 800 #Kg
mew = 1.7
g = 9.81 #m/s^2
a_traction_max = 8.0  # Based on motor power.
a_braking_max = mew * g
a_lateral_max = mew * g
v_max = 90 #m/s
v_limit = []


def corner_speed_limit(kappa):
    """Compute the maximum cornering speed at each point on the track."""
    kappa_0 = np.where(kappa == 0, 1e-9, kappa)  # avoid divide-by-zero
    v_limit = np.sqrt(a_lateral_max / np.abs(kappa_0))  # Speed at which the lateral acceleration reaches the grip limit.
    v_limit = np.minimum(v_limit, v_max)  # cap at absolute top speed
    return v_limit


def forward_pass(v_limit, ds, a_traction_max, v0):
    """Compute the acceleration-limited speed profile from start to finish."""
    v_forward = []
    for i in range(len(v_limit)):
        if i == 0:
            v_forward.append(v0)
        else:
            vel_forward = np.sqrt(v_forward[i-1]**2 + 2 * a_traction_max * ds)
            v_forward.append(min(vel_forward, v_limit[i]))
                
    return np.array(v_forward)


def backward_pass(v_limit, ds, a_braking_max, v_end):
    """Compute the braking-limited speed profile from finish to start."""
    v_backward = []
    for i in range(len(v_limit)-1, -1, -1):
        if i == len(v_limit)-1:
            v_backward.append(v_end)
        else:
            vel_backward = np.sqrt(v_backward[-1]**2 + 2 * a_braking_max * ds)  # Highest speed from which the car can still brake to the next point's speed.
            v_backward.append(min(vel_backward, v_limit[i]))
    v_backward.reverse()  # reverse the list to match the original order
    return np.array(v_backward)


def forward_pass_friction_circle(kappa, v_limit, ds, a_traction_max, a_lateral_max, v0):
    """Compute the acceleration-limited speed profile using a friction circle.

    Longitudinal acceleration is reduced by the lateral acceleration used in corners.
    """
    v_forward_friction_circle = []
    for i in range(len(v_limit)):
        if i == 0:
            v_forward_friction_circle.append(v0)
        else:
            a_lateral_used = min(a_lateral_max, v_forward_friction_circle[i-1]**2*abs(kappa[i-1]))
            a_long_tire =  mew*g *np.sqrt(1-(a_lateral_used/a_lateral_max)**2 )  # Longitudinal acceleration left from the tyres after cornering.
            a_long_available = min(a_long_tire, a_traction_max)  # Limited by either the tyres or the motor.
            v_possible = np.sqrt(v_forward_friction_circle[i-1]**2 + 2 * a_long_available * ds)
            v_forward_friction_circle.append(min(v_possible, v_limit[i]))
                
    return np.array(v_forward_friction_circle )


def backward_pass_friction_circle(kappa, v_limit, ds, a_braking_max, a_lateral_max, v_end):
    """Compute the braking-limited speed profile using a friction circle.

    Braking deceleration is reduced by the lateral acceleration used in corners.
    """
    v_backward_friction_circle = []
    for i in range(len(v_limit)-1, -1, -1):
        if i == len(v_limit)-1:
            v_backward_friction_circle.append(v_end)
        else:
            a_lateral_used = min(a_lateral_max, v_backward_friction_circle[-1]**2*abs(kappa[i+1]))
            a_long_available = a_braking_max*np.sqrt(1-(a_lateral_used/a_lateral_max)**2 )  # Braking deceleration left from the tyres after cornering.
            v_possible = np.sqrt(v_backward_friction_circle[-1]**2 + 2 * a_long_available * ds)
            v_backward_friction_circle.append(min(v_possible, v_limit[i]))
    v_backward_friction_circle.reverse()
    return np.array(v_backward_friction_circle)

def lap_time(v, ds=1.0):
    """Lap time (s) from the speed profile, using the average speed over each segment."""
    t = 0.0
    for i in range(len(v) - 1):   # Last point has no segment after it.
        v_avg = 0.5 * (v[i] + v[i + 1])
        t += ds / v_avg
    return t


def solve_simple(kappa, ds):
    """Run the simple model (separate limits) for a flying lap. Returns speed profile (m/s), lap time (s) and corner speed limits (m/s)."""
    v_limit = corner_speed_limit(kappa)
    # For a flying lap, the speed at the start line must equal the speed at the end of the lap.
    v0 = v_limit[0]      # initial guess: start at the speed limit
    v_end = v_limit[-1]  # initial guess: end at the speed limit
    for _ in range(2000):  # Limit the number of iterations to avoid infinite loops
        v_forward = forward_pass(v_limit, ds, a_traction_max, v0)
        v_backward = backward_pass(v_limit, ds, a_braking_max, v_end)
        v0_new = v_forward[-1]      # speed the car arrives back at the start line with
        v_end_new = v_backward[0]   # speed it can have at the start and still brake in time
        if abs(v0_new - v0) < 1e-6 and abs(v_end_new - v_end) < 1e-6:
            break
        v0 = v0_new
        v_end = v_end_new
    else:
        raise RuntimeError("v0 and v_end did not converge")
    v = np.minimum(v_forward, v_backward)   # The actual speed is the lowest of the two profiles.
    return v, lap_time(v, ds), v_limit


def solve_friction_circle(kappa, ds):
    """Run the friction circle model for a flying lap. Returns speed profile (m/s), lap time (s) and corner speed limits (m/s)."""
    v_limit = corner_speed_limit(kappa)
    # For a flying lap, the speed at the start line must equal the speed at the end of the lap.
    v0 = v_limit[0]      # initial guess: start at the speed limit
    v_end = v_limit[-1]  # initial guess: end at the speed limit
    for _ in range(2000):  # Limit the number of iterations to avoid infinite loops
        v_forward = forward_pass_friction_circle(kappa, v_limit, ds, a_traction_max, a_lateral_max, v0)
        v_backward = backward_pass_friction_circle(kappa, v_limit, ds, a_braking_max, a_lateral_max, v_end)
        v0_new = v_forward[-1]      # speed the car arrives back at the start line with
        v_end_new = v_backward[0]   # speed it can have at the start and still brake in time
        if abs(v0_new - v0) < 1e-6 and abs(v_end_new - v_end) < 1e-6:
            break
        v0 = v0_new
        v_end = v_end_new
    else:
        raise RuntimeError("v0 and v_end did not converge")
    v = np.minimum(v_forward, v_backward)
    return v, lap_time(v, ds), v_limit