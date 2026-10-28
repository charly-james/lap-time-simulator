import numpy as np
def build_track(segments, ds=1.0):
    distances = []
    total_distance = 0.0
    curvatures = []
    for seg_type, length, radius in segments:
        num_steps = int(length / ds)

        if seg_type == "transition":
            # Curvature ramps linearly from the start radius to the end radius (a clothoid). None = straight (kappa = 0).
            r_start, r_end = radius
            k_start = 0.0 if r_start is None else 1.0 / r_start
            k_end = 0.0 if r_end is None else 1.0 / r_end
            for j in range(num_steps):
                distances.append(total_distance)
                curvatures.append(k_start + (k_end - k_start) * j / num_steps)
                total_distance += ds
            continue   # go to the next segment

        if seg_type == "straight":
            curvature = 0.0
        elif seg_type == "corner":
            curvature = 1.0 / radius
        else:
            raise ValueError(f"Unknown segment type: {seg_type}")
        for _ in range(num_steps):
            distances.append(total_distance)
            curvatures.append(curvature)
            total_distance += ds
    # Close the lap: the last point is the start line again, so the segment back to the start is included.
    distances.append(total_distance)
    curvatures.append(curvatures[0])
    return np.array(distances), np.array(curvatures)

sample_circuit = [
    ("straight", 300, None),
    ("transition", 30, (None, 40)),
    ("corner", 33, 40),
    ("transition", 30, (40, None)),
    ("straight", 200, None),
    ("transition", 30, (None, 120)),
    ("corner", 96, 120),
    ("transition", 30, (120, None)),
    ("straight", 448, None),
    ("transition", 30, (None, -60)),    
    ("corner", 17, -60),
    ("transition", 30, (-60, None)),
    ("straight", 150, None),
    ("transition", 30, (None, 25)),
    ("corner", 35, 25),
    ("transition", 30, (25, None)),
    ("straight", 560, None),
    ("transition", 30, (None, 199.3)),
    ("corner", 337, 199.3),
    ("transition", 30, (199.3, None)),
    ("straight", 200, None),
]