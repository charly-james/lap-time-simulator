import numpy as np
from scipy.optimize import brentq

def speed_limit(vehicle, kappa):
    """Compute the maximum speed at each point on the track."""

    v_limit = np.zeros_like(kappa)
    for i in range(len(kappa)):

        if kappa[i] == 0: # Straight line so no limit from lateral acceleration
            v_limit[i] = vehicle.v_max()
            continue 
        # In a corner, we need to find the speed corresponding to the maximum ay.
        # Therefore, we need to solve the equation ay_grip_max(v) - v^2 * |kappa| = 0 for v.

        def f(v):
            return vehicle.ay_grip_max(v)-v**2*abs(kappa[i])

        # If there is still grip available at the top speed, then the limit is the top speed.
        if f(vehicle.v_max())>=0:
            v_limit[i] = vehicle.v_max()
        else:
            # Otherwise, we need to solve f(v) = 0 for v.
            # we use the brentq method to find the root of f(v) between 0 and the top speed, vehicle.v_max().
            v_limit[i] = brentq(f, 0, vehicle.v_max())
    return v_limit

def forward_pass(vehicle, v_limit, v0, kappa, ds):
    """Compute the acceleration-limited speed profile from start to finish using the bicycle model grip limits."""
    v_forward = np.zeros_like(v_limit)
    v_forward[0] = v0

    for i in range (len(kappa)-1):
       ax = vehicle.a_power_max(v_forward[i]) # initial guess for the longitudinal acceleration.
       for j in range(2000):  # Limit the number of iterations to avoid infinite loops
            ax_grip_max = vehicle.ax_grip_max(v_forward[i], ax, kappa[i])
            a_power_max = vehicle.a_power_max(v_forward[i])
            ax_new = min(ax_grip_max, a_power_max) - vehicle.total_resistance(v_forward[i])/vehicle.mass
            if abs(ax_new - ax) <= 1e-6:
                break
            ax = ax_new
       else:
            raise RuntimeError(f"ax did not converge at point {i}, v = {v_forward[i]:.2f} m/s")
       v_forward[i+1] = min(v_limit[i+1], np.sqrt(v_forward[i]**2 + 2 * ax_new * ds))
    return v_forward

def backward_pass(vehicle, v_limit, v_end, kappa, ds):
    """Compute the braking-limited speed profile, going backwards from the end of the track to the start."""
    v_backward = np.zeros_like(v_limit)
    v_backward[-1] = v_end

    for i in range(len(kappa)-1, 0, -1):
        v = v_backward[i]

        # Bisection on the deceleration d: the car can brake at d if the tyres can deliver the braking d requires,
        # with the axle loads that d creates (ax = -d because the car is slowing down).
        d_low = vehicle.total_resistance(v)/vehicle.mass   # no tyre braking, only drag and rolling resistance: always possible
        d_high = 50.0              # about 5 g: always impossible for this car
        while d_high - d_low > 1e-6:
            d_mid = (d_low + d_high)/2
            tyre_braking = vehicle.a_braking_max(v, -d_mid, kappa[i])
            if tyre_braking + vehicle.total_resistance(v)/vehicle.mass >= d_mid:
                d_low = d_mid    # possible: the max is at least d_mid
            else:
                d_high = d_mid   # not possible: the max is below d_mid
        d = d_low

        # Going backwards, braking makes the speed grow: v_prev^2 = v^2 + 2*d*ds.
        v_backward[i-1] = min(v_limit[i-1], np.sqrt(v**2 + 2*d*ds))

    return v_backward

def lap_time(v, ds):
    """Lap time (s) from the speed profile, using the average speed over each segment."""
    t = 0.0
    for i in range(len(v) - 1):   # Last point has no segment after it.
        v_avg = 0.5 * (v[i] + v[i + 1])
        t += ds / v_avg
    return t

def solve(vehicle, kappa, ds):
    """Solve a flying lap: returns the speed profile, the lap time and the speed limits."""

    v_limit = speed_limit(vehicle, kappa)

    # For a flying lap we need to find v0 and v_end such that the forward and backward passes meet.
    # We can do this by iterating on v0 and v_end until they converge.

    v0 = v_limit[0]  # initial guess: start at the speed limit
    v_end = v_limit[-1]  # initial guess: end at the speed limit

    for _ in range(2000):  # Limit the number of iterations to avoid infinite loops
        v_forward = forward_pass(vehicle, v_limit, v0, kappa, ds)
        v_backward = backward_pass(vehicle, v_limit, v_end, kappa, ds)
        v0_new = v_forward[-1]
        v_end_new = v_backward[0]

        if abs(v0_new - v0) < 1e-6 and abs(v_end_new - v_end) < 1e-6:
            break
        v0 = v0_new
        v_end = v_end_new

    else:
        raise RuntimeError("v0 and v_end did not converge")
    v = np.minimum(v_forward, v_backward)
    return v, lap_time(v, ds), v_limit