import numpy as np


def corner_speed_limit(kappa, vehicle):
    """
    Max steady-state cornering speed with downforce:
        v^2 * |kappa| = mu * (g + rho*ClA*v^2 / (2m))
        v = sqrt(mu*g / (|kappa| - c)), with c = mu*rho*ClA / (2m)
    If |kappa| <= c, downforce grows faster than the demand, therefore there is no speed limit (v_max is the limit).
    
    """
    v_limit = []
    k = np.abs(np.asarray(kappa, dtype=float))
    c = vehicle.mew * vehicle.rho * vehicle.ClA / (2 * vehicle.mass)
    v_max = vehicle.v_max()
    for i in range(len(k)):
        if k[i]<= c:    # Case where there is no limit due to downforce.
            v_limit.append(v_max) 
        else:
            v_limit.append(np.minimum(np.sqrt(vehicle.mew * vehicle.g / (k[i] - c)), v_max))  # cap at absolute top speed
    speed_limit = np.array(v_limit)
    return speed_limit


def a_long_tyre(v, kappa, vehicle):
    """Longitudinal tyre acceleration left over after cornering (friction circle)."""
    a_grip_max = vehicle.a_grip_max(v)
    a_lat_used = min(a_grip_max, v**2 * abs(kappa)) # Caps lateral acceleration at the maximum available grip.
    return np.sqrt(a_grip_max**2 - a_lat_used**2)


def forward_pass(kappa, v_limit, ds, vehicle, v_0=0.0):
    """Acceleration-limited speed profile (m/s), from start to finish. ds in m."""
    v = np.zeros(len(v_limit))
    v[0] = v_0
    for i in range(1, len(v_limit)):
        v_prev = v[i - 1] 
        a_tyre = a_long_tyre(v_prev, kappa[i - 1], vehicle)
        a_drive = min(vehicle.a_power_max(v_prev), a_tyre)   # Limited by either the power or the tyres.
        a_net = a_drive - vehicle.total_resistance(v_prev) / vehicle.mass  # Net acceleration accounting for drag and rolling resistance.
        v_sq = max(v_prev**2 + 2 * a_net * ds, 0.0) # Speed squared can't go negative (car stopped).
        v[i] = min(np.sqrt(v_sq), v_limit[i])
    return v


def backward_pass(kappa, v_limit, ds, vehicle):
    """Braking-limited speed profile (m/s), from finish to start. ds in m."""
    n = len(v_limit)
    v = np.zeros(n)
    v[-1] = v_limit[-1]
    for i in range(n - 2, -1, -1):
        v_next = v[i + 1]
        a_tyre = a_long_tyre(v_next, kappa[i + 1], vehicle)
        a_brake = a_tyre + vehicle.total_resistance(v_next) / vehicle.mass      # Drag and rolling resistance help the car brake.
        v[i] = min(np.sqrt(v_next**2 + 2 * a_brake * ds), v_limit[i])
    return v


def lap_time(v, ds):
    """Lap time (s) from the speed profile, using the average speed over each segment."""
    t = 0.0
    for i in range(len(v) - 1):   # Last point has no segment after it.
        v_avg = 0.5 * (v[i] + v[i + 1])
        t += ds / v_avg
    return t


def solve(kappa, ds, vehicle, v_0=0.0):
    """Run the full lap simulation. Returns speed profile (m/s), lap time (s) and corner speed limits (m/s)."""
    v_limit = corner_speed_limit(kappa, vehicle)
    v_fwd = forward_pass(kappa, v_limit, ds, vehicle, v_0)
    v_bwd = backward_pass(kappa, v_limit, ds, vehicle)
    v = np.minimum(v_fwd, v_bwd)
    return v, lap_time(v, ds), v_limit