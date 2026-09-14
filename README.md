# Lap Time Simulator

A Python-based vehicle lap time simulator built from first principles, applying
vehicle dynamics to model acceleration, braking, and cornering performance.

## Overview

This project simulates a vehicle's speed around a track by combining:
- A free-body derivation of a vehicle's traction, braking, and cornering limits
- A point-mass vehicle model with a combined friction-ellipse tyre constraint
- A forward/backward speed-profile solver to compute a physically achievable
  speed trace and resulting lap time

## Current features

- Track definition from straight/corner segments, with curvature computed
  at fine resolution along the lap
- Corner speed limits derived from a friction-circle tyre model
- Forward and backward acceleration/braking passes, combined to produce a
  realistic speed profile (respecting the fact that braking must begin
  before a corner, not at its boundary)
- Combined friction-ellipse constraint, coupling lateral and longitudinal
  tyre force rather than treating them independently
- Lap time integration from the resulting speed profile

## Roadmap

- [x] Point-mass simulator with friction-circle tyre model
- [x] Forward/backward speed-profile solver
- [x] Combined friction ellipse (lateral/longitudinal coupling)
- [ ] Pacejka Magic Formula tyre model (in progress)
- [ ] Combined-slip Magic Formula
- [ ] Brush-model tyre force derivation and implementation
- [ ] Validation against real track lap time data
- [ ] Aerodynamic and suspension effects

## Project structure
track.py - track definition and curvature calculation
vehicle.py - vehicle parameters, tyre limits, and forward/backward solver
main.py - runs a full simulation and plots the results
## Running it

```bash
git clone https://github.com/charly-james/lap-time-simulator.git
cd lap-time-simulator
python -m venv venv
venv\Scripts\activate          # Windows
pip install -r requirements.txt
python main.py
```

## Background

This project grew out of self-teaching vehicle dynamics and tyre mechanics
(brush model, Pacejka Magic Formula, combined slip) from first principles,
and is being developed incrementally as a way to apply and test that
understanding against real physics and, eventually, real lap time data.