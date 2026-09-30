import numpy as np
import matplotlib.pyplot as plt

from track import build_track, sample_circuit
from vehicle import Vehicle
from solver import solve
ds = 1.0
vehicle = Vehicle()
[s, kappa] = build_track(sample_circuit, ds)
[v, lap_time, v_limit] = solve(kappa, ds, vehicle, v_0=0.0)
print(f"Lap time: {lap_time:.2f} seconds")

plt.figure()
plt.plot(s,v, label="speed profile", alpha=0.8)
plt.plot(s, v_limit, linestyle = "--", label="speed limit", alpha=0.8)
plt.xlabel("Distance (m)")
plt.ylabel("Speed (m/s)")
plt.title("v1.5: speed profile")
plt.legend()
plt.grid(True)
plt.show()