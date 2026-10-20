# Lap Time Simulator

A Python-based vehicle lap time simulator built from first principles, applying
vehicle dynamics to model velocity, acceleration, braking, and cornering performance.

## Overview

This project simulates a vehicle's speed around a track by combining:
- A free-body derivation of a vehicle's traction, braking, and cornering limits
- A point-mass vehicle model with a combined friction-circle tyre constraint
- A steady-state bicycle model with per-axle grip, longitudinal load transfer
  and load-sensitive tyres
- A forward/backward speed-profile solver to compute a physically achievable
  speed trace and resulting lap time
- The effects of downforce, drag, and rolling resistance.

The project is developed in versions, each kept in its own folder so the
progression can be followed.

## Versions

## Versions

### v1 - Point-mass model with friction circle
- Track built from straight and corner segments, split into 1 m steps, each with its curvature (1/radius, zero on straights)
- Corner speed limit from the tyre grip: v = √(μg/κ), capped at a fixed top speed
- Forward pass (acceleration) and backward pass (braking), combined by taking the lower speed at each point
- Two models compared:
  - Separate limits: fixed maximum acceleration, braking and cornering, each treated independently
  - Friction circle: acceleration, braking and cornering share the same tyre grip, so the car cannot brake or accelerate fully while cornering

### v1.5 - Powertrain and aerodynamic forces
- Engine limit: constant maximum drive force below the motor's base speed, constant power above it (F = P/v)
- Aerodynamic drag, downforce and rolling resistance
- Downforce adds tyre load, so the corner speed limit rises with speed; it is still solved exactly with a formula
- Top speed computed from the power–drag balance instead of a fixed cap
- Lap time calculated using the average speed over each segment
- Vehicle parameters grouped in a `Vehicle` dataclass

### v2 - Bicycle model with load-sensitive tyres
- Steady-state bicycle model: the lateral force each axle must produce is set by the cornering acceleration and the CG position (Fyf = m·ay·b/L, Fyr = m·ay·a/L), so the car is limited by whichever axle saturates first
- Axle loads come from the static weight distribution, the downforce split by the aero balance, and longitudinal load transfer (m·ax·h/L)
- Load-sensitive tyres: the friction coefficient μ decreases linearly with tyre load
- Per-axle friction ellipse: each axle shares its grip between cornering and braking or accelerating
- Rear-wheel drive, and braking split between the axles by the brake bias
- Front axle check under acceleration: accelerating unloads the front, which still has to hold its share of the cornering force
- Corner speed limits found numerically with Brent's method: with load sensitivity, the equation can no longer be solved with a formula
- Load transfer and grip depend on each other: the acceleration moves load between the axles, which changes the grip, which changes the maximum acceleration. Under acceleration this is solved by fixed-point iteration (started from the engine limit); under braking by bisection, because the fixed-point iteration sometimes does not converge there

## Results

Sample circuit: 1800 m, flying lap (All lap times are for a flying lap: the start speed is found so that the car crosses the line at the same speed it started with.):

| Model | Lap time |
|---|---|
| v1 - separate limits | 37.72 s |
| v1 - friction circle | 37.86 s |
| v1.5 | 41.53 s |
| v2 | 41.84 s |

![Speed trace v1](v1_results.png)
![Speed trace v1.5](v1.5_results.png)
![Speed trace v2](v2_results.png)

In v1 the car accelerates at a constant rate. In v1.5 the acceleration drops as speed rises, because above the motor's base speed the drive force falls as P/v while drag grows with v², so the speed levels off towards a top speed set by power and drag rather than by a fixed cap.

The car can also brake later in v1.5, since drag helps slow it down and downforce adds grip at high speed.

Downforce raises the corner speeds since it increases the normal force and in consequence, the maximum grip, and because it grows with v², the gain is much bigger in fast corners than in slow ones. As a result, in the second turn, the car stays below its cornering limit, so it is limited by power rather than grip. In the last turn, the grip limit allows a speed even higher than the car's top speed, so the corner doesn't limit the car at all.

From v1.5 to v2, the straights and fast corners are almost unchanged, since both versions use the same power, drag and downforce. Most of the difference comes from the two slow corners, where v2's corner speeds are lower, so the car brakes earlier and accelerates out from a lower speed. Three effects are new in v2. First, each axle has its own grip limit instead of the grip being pooled for the whole car, so the car is limited by whichever axle saturates first. Second, longitudinal load transfer: accelerating moves load to the rear axle and braking moves it to the front. Third, tyre friction now decreases with load.

To measure each effect, `compare.py` turns them off and reruns the lap (the v1.5 row comes from `v1.5/main.py`):

| Run | Lap time | R40 corner limit (40 m radius). |
|---|---|---|
| v1.5 | 41.53 s | 28.12 m/s |
| v2 | 41.84 s | 27.37 m/s |
| v2, no load transfer (h = 0) | 42.07 s | 27.37 m/s |
| v2, no load sensitivity (k_load = 0) | 41.59 s | 28.05 m/s |
| v2, axle split only (h = 0, k_load = 0) | 41.79 s | 28.05 m/s |
| v2, axle split only, brake bias 43 % | 41.55 s | 28.05 m/s |

- **Axle split:** it costs almost no cornering speed (28.12 → 28.05 m/s), because the aero balance (42 % front) is close to the static weight distribution (43 % front). But on its own it makes the lap 0.26 s slower, and this time is lost under braking: the brake bias sends 58 % of the braking to the front, while without load transfer the front only carries 43 % of the weight, so the front tyres reach their limit first.  With the brake bias set to 43 %, equal to the front axle's share of the weight, the lap drops back to 41.55 s, almost the v1.5 time: the axle split itself costs almost nothing, the time comes from the brake bias not matching the axle loads.
- **Load transfer:** it does not change the corner limits, since the car is not accelerating at the apex, but it makes the car faster under braking and acceleration: under braking it loads the front axle, which takes 58 % of the braking, and under acceleration it loads the driven rear axle. Adding it to the axle split gains 0.20 s, and removing it from v2 costs 0.23 s.
- **Load sensitivity:** the more heavily loaded rear tyres have a lower friction coefficient than the fronts, and since the car is limited by the weaker axle, this lowers the cornering limit of the whole car (28.05 → 27.37 m/s), costing 0.24 s. It costs much more cornering speed than the axle split, and is the reason v2's corner speeds are lower.


## Limitations

- v1 and v1.5 are point-mass models: no yaw dynamics, steering, tyre slip angles or load transfer
- v2 is quasi-steady-state: the car is assumed to be in steady cornering at each point, so yaw transients (corner entry and exit) are ignored, which makes lap times slightly optimistic
- The bicycle model lumps each axle into a single tyre, so lateral load transfer is not modelled
- Linear load sensitivity law, with the same μ in the lateral and longitudinal directions
- Combined slip uses a friction ellipse per axle rather than the Magic Formula combined-slip functions
- Drag acts at the CG height in the load transfer, so its pitching moment is ignored; aero balance is constant with speed
- Vehicle parameters are placeholders, not a specific real car

## Roadmap

- [x] v1: Point-mass simulator with friction-circle tyre model and forward/backward speed-profile solver
- [x] v1.5: Power-limited traction (constant-force and constant-power regions)
- [x] v1.5: Aerodynamic drag, downforce and rolling resistance
- [x] v2: Bicycle model with per-axle grip, longitudinal load transfer and load-sensitive tyres
- [ ] v2: Magic Formula outputs: slip angles, steering angle and body slip along the lap
- [ ] Lateral load transfer, with front/rear roll stiffness distribution
- [ ] Real circuits imported from centreline coordinates (CSV), with curvature computed from x-y data
- [ ] Validation against real track lap time data
- [ ] Suspension effects
## Project structure

v1/
track.py - track definition and curvature calculation
vehicle.py - vehicle parameters, tyre limits, and forward/backward solver
main.py - runs both models and plots the results
v1.5/
track.py - track definition and curvature calculation
vehicle.py - Vehicle dataclass: forces and acceleration limits
solver.py - corner limits, forward/backward passes and lap time
main.py - runs a full simulation and plots the results
V2/
Track.py - track definition and curvature calculation
Vehicle.py - Vehicle dataclass: axle loads, load-sensitive tyres and per-axle grip limits
Solver.py - corner limits, forward/backward passes with load transfer, flying lap and lap time
Main.py - runs a full simulation and plots the results
Compare.py - turns off load transfer and load sensitivity one at a time to measure their effect


## Running it

```bash
git clone https://github.com/charly-james/lap-time-simulator.git
cd lap-time-simulator
python -m venv venv
venv\Scripts\activate          # Windows
pip install -r requirements.txt
cd V2                          # or cd v1.5, cd v1
python Main.py
```
