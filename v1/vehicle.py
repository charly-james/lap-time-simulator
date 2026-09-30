"""Point-mass lap time simulator (v1): vehicle parameters and speed profile solver."""

import numpy as np

# Known variables:
mass = 800 #Kg
mew = 1.5
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


def forward_pass(v_limit, ds, a_traction_max, v_0=0.0):
    """Compute the acceleration-limited speed profile from start to finish."""
    v_forward = []
    for i in range(len(v_limit)):
        if i == 0:
            v_forward.append(v_0)
        else:
            vel_forward = np.sqrt(v_forward[i-1]**2 + 2 * a_traction_max * ds)
            v_forward.append(min(vel_forward, v_limit[i]))
                
    return np.array(v_forward)


def backward_pass(v_limit, ds, a_braking_max):
    """Compute the braking-limited speed profile from finish to start."""
    v_backward = []
    for i in range(len(v_limit)-1, -1, -1):
        if i == len(v_limit)-1:
            v_backward.append(v_limit[i])
        else:
            vel_backward = np.sqrt(v_backward[-1]**2 + 2 * a_braking_max * ds)  # Highest speed from which the car can still brake to the next point's speed.
            v_backward.append(min(vel_backward, v_limit[i]))
    v_backward.reverse()  # reverse the list to match the original order
    return np.array(v_backward)


def forward_pass_ellipse(kappa, v_limit, ds, a_traction_max, a_lateral_max, v_0=0.0):
    """Compute the acceleration-limited speed profile using a friction ellipse.

    Longitudinal acceleration is reduced by the lateral acceleration used in corners.
    """
    v_forward_ellipse = []
    for i in range(len(v_limit)):
        if i == 0:
            v_forward_ellipse.append(v_0)
        else:
            a_lateral_used = min(a_traction_max, v_forward_ellipse[i-1]**2*abs(kappa[i-1]))
            a_long_tire =  mew*g *np.sqrt(1-(a_lateral_used/a_lateral_max)**2 )  # Longitudinal acceleration left from the tyres after cornering.
            a_long_available = min(a_long_tire, a_traction_max)  # Limited by either the tyres or the motor.
            v_possible = np.sqrt(v_forward_ellipse[i-1]**2 + 2 * a_long_available * ds)
            v_forward_ellipse.append(min(v_possible, v_limit[i]))
                
    return np.array(v_forward_ellipse )


def backward_pass_ellipse(kappa, v_limit, ds, a_braking_max, a_lateral_max):
    """Compute the braking-limited speed profile using a friction ellipse.

    Braking deceleration is reduced by the lateral acceleration used in corners.
    """
    v_backward_ellipse = []
    for i in range(len(v_limit)-1, -1, -1):
        if i == len(v_limit)-1:
            v_backward_ellipse.append(v_limit[i])
        else:
            a_lateral_used = min(a_braking_max, v_backward_ellipse[-1]**2*abs(kappa[i+1]))
            a_long_available = a_braking_max*np.sqrt(1-(a_lateral_used/a_lateral_max)**2 )  # Braking deceleration left from the tyres after cornering.
            v_possible = np.sqrt(v_backward_ellipse[-1]**2 + 2 * a_long_available * ds)
            v_backward_ellipse.append(min(v_possible, v_limit[i]))
    v_backward_ellipse.reverse()
    return np.array(v_backward_ellipse)

    
def lap_time(v_actual, ds=1.0):
    """Compute the total lap time from the speed profile."""
    n_zeros = np.sum(v_actual == 0)
    v_safe = np.where(v_actual == 0, 1, v_actual)   # Replace zero speeds to avoid dividing by zero.
    time_per_step = ds / v_safe
    return np.sum(time_per_step) - ds * n_zeros   # Remove the steps where the speed was zero.


def solve_simple(kappa, ds, v_0=0.0):
    """Run the simple model (separate limits). Returns speed profile (m/s), lap time (s) and corner speed limits (m/s)."""
    v_limit = corner_speed_limit(kappa)
    v_fwd = forward_pass(v_limit, ds, a_traction_max, v_0)
    v_bwd = backward_pass(v_limit, ds, a_braking_max)
    v = np.minimum(v_fwd, v_bwd)   # The actual speed is the lowest of the two profiles.
    return v, lap_time(v, ds), v_limit


def solve_ellipse(kappa, ds, v_0=0.0):
    """Run the friction ellipse model. Returns speed profile (m/s), lap time (s) and corner speed limits (m/s)."""
    v_limit = corner_speed_limit(kappa)
    v_fwd = forward_pass_ellipse(kappa, v_limit, ds, a_traction_max, a_lateral_max, v_0)
    v_bwd = backward_pass_ellipse(kappa, v_limit, ds, a_braking_max, a_lateral_max)
    v = np.minimum(v_fwd, v_bwd)
    return v, lap_time(v, ds), v_limit