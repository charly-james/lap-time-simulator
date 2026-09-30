import numpy as np
import matplotlib.pyplot as plt

from track import build_track, sample_circuit
from vehicle import solve_simple, solve_ellipse

ds = 1.0

s, kappa = build_track(sample_circuit, ds)

v_simple, t_lap_simple, v_limit = solve_simple(kappa, ds)
print(f"Lap time for the simple model: {t_lap_simple:.2f} seconds")

v_ellipse, t_lap_ellipse, _ = solve_ellipse(kappa, ds)
print(f"Lap time ellipse: {t_lap_ellipse:.2f} seconds")

# --- Comparison ---
pct_diff = (t_lap_ellipse - t_lap_simple) / t_lap_simple * 100
print(f"Percentage difference in lap times: {pct_diff:.2f}%")

plt.figure()
plt.plot(s, v_simple, label="separate limits", alpha=0.8)
plt.plot(s, v_ellipse, linestyle="--", label="friction ellipse", alpha=0.8)
plt.plot(s, v_limit, linestyle="--", label="speed limit", alpha=0.8)
plt.xlabel("Distance along track (m)")
plt.ylabel("Speed (m/s)")
plt.title("Speed Profile: separate limits vs. friction ellipse")
plt.grid(True)
plt.legend()
plt.show()