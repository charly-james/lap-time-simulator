import numpy as np
import matplotlib.pyplot as plt

from track import build_track, sample_circuit
from vehicle import solve_simple, solve_friction_circle

ds = 1.0

s, kappa = build_track(sample_circuit, ds)

v_simple, t_lap_simple, v_limit = solve_simple(kappa, ds)
print(f"Lap time for the simple model: {t_lap_simple:.2f} seconds")

v_friction_circle, t_lap_friction_circle, _ = solve_friction_circle(kappa, ds)
print(f"Lap time friction circle: {t_lap_friction_circle:.2f} seconds")

# --- Comparison ---
pct_diff = (t_lap_friction_circle - t_lap_simple) / t_lap_simple * 100
print(f"Percentage difference in lap times: {pct_diff:.2f}%")

plt.figure()
plt.plot(s, v_simple, label="separate limits", alpha=0.8)
plt.plot(s, v_friction_circle, linestyle="--", label="friction circle", alpha=0.8)
plt.plot(s, v_limit, linestyle="--", label="speed limit", alpha=0.8)
plt.xlabel("Distance along track (m)")
plt.ylabel("Speed (m/s)")
plt.title("v1: Speed Profile: separate limits vs. friction circle")
plt.grid(True)
plt.legend()
plt.show()