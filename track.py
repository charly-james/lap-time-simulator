import numpy as np
def build_track(segments, ds=1.0):
    distances = []
    total_distance = 0.0
    curvatures = []
    for seg_type, length, radius in segments:
        num_steps = int(length / ds)
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
    return np.array(distances), np.array(curvatures)
sample_circuit = [("straight", 400, None),
    ("corner", 90, 40),      # tight hairpin
    ("straight", 250, None),
    ("corner", 150, 120),    # fast sweeper
    ("straight", 180, None),
    ("corner", 60, 25),      # very tight chicane-like corner
    ("straight", 320, None),
    ("corner", 200, 200),    # gentle, high-speed corner
    ("straight", 150, None),]
    
