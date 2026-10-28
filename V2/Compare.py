"""Effect of each new v2 feature on the lap time and on the corner speed limit.

Each run turns off one feature of the full v2 model and compares it with the full model:
- no load transfer: CG height h = 0, so accelerating or braking does not move load between the axles
- no load sensitivity: k_load = 0, so the tyre friction coefficient no longer depends on load
- axle split only: both h = 0 and k_load = 0, so the tyres are limited by the axle load split only, with no load transfer or load sensitivity
- axle split only, brake bias 43 %: same as above, but with a brake bias simiar to the front/rear weight distribution

The lap time and the speed limit in the 40 m-radius corner are printed for each run
"""
from dataclasses import replace
import warnings

from Track import build_track, sample_circuit
from Vehicle import Vehicle
from Solver import solve



ds = 1.0
s, kappa = build_track(sample_circuit, ds)
corner = (s >= 400) & (s <= 490)    # the 40 m-radius corner

runs = {
    "v2 (full model)": Vehicle(),
    "no load transfer (h = 0)": replace(Vehicle(), h=0.0),
    "no load sensitivity (k_load = 0)": replace(Vehicle(), k_load=0.0),
    "axle split only (h = 0, k_load = 0)": replace(Vehicle(), h=0.0, k_load=0.0),
    "axle split only, brake bias 43 %": replace(Vehicle(), h=0.0, k_load=0.0, brake_bias=0.43),
}

print(f"{'Run':<35}{'Lap time (s)':>14}{'R40 limit (m/s)':>18}")
for name, vehicle in runs.items():
    v, lap_time, v_limit, ax = solve(vehicle, kappa, ds)
    print(f"{name:<35}{lap_time:>14.2f}{v_limit[corner].min():>18.2f}")