import numpy as np
import matplotlib.pyplot as plt

from track import build_track, sample_circuit
from vehicle import backward_pass, corner_speed_limit, a_traction_max, a_braking_max, forward_pass, forward_pass_ellipse, backward_pass_ellipse, lap_time,a_lateral_max
backward_pass

ds = 1.0

s, kappa = build_track(sample_circuit, ds)
v_limit = corner_speed_limit(kappa)


v_forward_simple = forward_pass(v_limit, ds, a_traction_max, v_0=0.0)
v_backward_simple = backward_pass(v_limit, ds, a_braking_max)
v_simple = np.minimum(v_forward_simple, v_backward_simple)
t_lap_simple = lap_time(v_simple, ds)
print(f"Lap time for the simple model: {t_lap_simple:.2f} seconds")

v_forward_ellipse = forward_pass_ellipse(kappa,v_limit, ds, a_traction_max,a_lateral_max, v_0=0.0)
v_backward_ellipse = backward_pass_ellipse(kappa, v_limit, ds, a_braking_max, a_lateral_max)
v_ellipse = np.minimum(v_forward_ellipse, v_backward_ellipse)  
t_lap_ellipse = lap_time(v_ellipse, ds)
print(f"Lap time ellipse: {t_lap_ellipse:.2f} seconds")

# --- Comparison ---
pct_diff = (t_lap_ellipse - t_lap_simple) / t_lap_simple * 100
print(f"Percentage difference in lap times: {pct_diff:.2f}%")

plt.figure()
plt.plot(s, v_simple, label="separate limits", alpha=0.8)
plt.plot(s, v_ellipse,linestyle="--", label="friction ellipse", alpha=0.8)
plt.xlabel("Distance along track (m)")
plt.ylabel("Speed (m/s)")
plt.title("Speed Profile: separate limits vs. friction ellipse")
plt.legend()
plt.show()