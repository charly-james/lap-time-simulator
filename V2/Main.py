import numpy as np
import matplotlib.pyplot as plt

from Track import build_track, sample_circuit
from Vehicle import Vehicle
from Solver import solve
ds = 1.0
vehicle = Vehicle()
[s, kappa] = build_track(sample_circuit, ds)
[v, lap_time, v_limit] = solve(vehicle, kappa, ds)
print(f"Lap time: {lap_time:.2f} seconds")

plt.figure()
plt.plot(s,v, label="speed profile", alpha=0.8)
plt.plot(s, v_limit, linestyle = "--", label="speed limit", alpha=0.8)
plt.xlabel("Distance (m)")
plt.ylabel("Speed (m/s)")
plt.title("V2: speed profile")
plt.legend()
plt.grid(True)
plt.show()