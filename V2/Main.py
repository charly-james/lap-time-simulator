import numpy as np
import matplotlib.pyplot as plt

from Track import build_track, sample_circuit
from Vehicle import Vehicle
from Solver import solve
from Outputs import outputs
ds = 1.0
vehicle = Vehicle()
[s, kappa] = build_track(sample_circuit, ds)
[v, lap_time, v_limit, ax] = solve(vehicle, kappa, ds)
alpha_f = np.zeros_like(kappa)
alpha_r = np.zeros_like(kappa)
ay = np.zeros_like(kappa)
steering_angle = np.zeros_like(kappa)
for i in range(len(kappa)):
    alpha_f[i], alpha_r[i], steering_angle[i], ay[i] = outputs(vehicle, kappa[i], v[i], ax[i])
print(f"Lap time: {lap_time:.2f} seconds")

fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7))   # 2 rows, 1 column, same x-axis (distance)


axes[0].plot(s, v, label="speed profile")
axes[0].plot(s, v_limit, linestyle="--", label="speed limit")
axes[0].set_ylabel("Speed (m/s)")
axes[0].legend()
axes[0].grid(True)


axes[1].plot(s, np.degrees(alpha_f), label="front slip angle")
axes[1].plot(s, np.degrees(alpha_r), linestyle="--", label="rear slip angle")
axes[1].plot(s, np.degrees(steering_angle), linestyle = "--", label="steering angle")
axes[1].set_ylabel("Slip angle (deg)")
axes[1].set_xlabel("Distance (m)")
axes[1].legend()
axes[1].grid(True)



fig.tight_layout()   # stops the labels overlapping

# GG diagram: lateral vs longitudinal acceleration, coloured by speed
g = vehicle.g
fig_gg, ax_gg = plt.subplots(figsize=(6, 6))

points = ax_gg.scatter(ay/g, ax/g, c=v, cmap="viridis", s=8)   # one dot per track point, colour = speed
fig_gg.colorbar(points, ax=ax_gg, label="Speed (m/s)")

ax_gg.axhline(0, color="grey", linewidth=0.8)   # zero lines to split braking/accelerating and left/right
ax_gg.axvline(0, color="grey", linewidth=0.8)
ax_gg.set_xlabel("Lateral acceleration ay (g)")
ax_gg.set_ylabel("Longitudinal acceleration ax (g)")
ax_gg.set_title("GG diagram")
ax_gg.set_aspect("equal")    # same scale on both axes, so the friction ellipse isn't distorted
ax_gg.grid(True)

plt.show()