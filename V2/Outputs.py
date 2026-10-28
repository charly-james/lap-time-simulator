import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import brentq

from Track import build_track, sample_circuit
from Vehicle import Vehicle
from Solver import solve
ds = 1.0
vehicle = Vehicle()
[s, kappa] = build_track(sample_circuit, ds)
[v, lap_time, v_limit, ax] = solve(vehicle, kappa, ds)

def slip_angle(vehicle, F_required, Fz):
    """Slip angle (rad) the axle needs to produce the required lateral force at load Fz."""
    F = abs(F_required)
    D = vehicle.mew(Fz)*Fz                   # peak lateral force of the axle

    if F == 0:
        return 0.0                           # straight: no lateral force needed
    if F >= D*(1 - 1e-9): 
        return np.sign(F_required)*vehicle.peak_slip_angle       # at (or a rounding error above) the grip limit

    def f(alpha):
        return vehicle.magic_formula_lateral(Fz, alpha) - F

    # Rising part of the curve only: exactly one solution between 0 and the peak.
    return np.sign(F_required)*brentq(f, 0, vehicle.peak_slip_angle)

def outputs(vehicle, kappa, v, ax):
     ay = v**2 * kappa
     Fyf = vehicle.mass * ay * vehicle.b / vehicle.L
     Fyr = vehicle.mass * ay * vehicle.a / vehicle.L
     Fzf, Fzr = vehicle.normal_force(v, ax)
     alpha_f = slip_angle(vehicle, Fyf, Fzf)
     alpha_r = slip_angle(vehicle, Fyr, Fzr)
     steering_angle = vehicle.L*kappa + alpha_f - alpha_r
     return alpha_f, alpha_r, steering_angle, ay
