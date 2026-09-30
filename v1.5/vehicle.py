from dataclasses import dataclass
import numpy as np

@dataclass
class Vehicle:
    mass: float = 800.0  # kg
    mew: float = 1.5     #tyre friction coefficient
    g: float = 9.81      # m/s^2
    power: float = 150e3    # W (peak power at the wheels)
    rho: float = 1.225   # kg/m^3 (air density)
    CdA: float =1.2      # m^2, drag coefficient x frontal area
    ClA: float = 3.0      # m^2, lift (downforce) coefficient x area
    Crr: float = 0.015       # rolling resistance coefficient
    F_drive_max: float = 6400.0  # N, maximum drive force at the wheels (limited by maximum torque and wheel radius)

    # Forces:

    def downforce(self, v):
        return 0.5 * self.rho * self.ClA * v**2

    def drag(self, v):
        return 0.5 * self.rho * self.CdA * v**2

    def normal_force(self, v):
        return self.mass * self.g + self.downforce(v)

    def rolling_resistance(self, v):
        return self.Crr * self.normal_force(v)
    
    def total_resistance(self, v):
        return self.drag(v) + self.rolling_resistance(v)
    
    # Acceleration limits:

    def a_power_max(self, v):
        """Calculate the base velocity before which the torque is constant and after which the torque is limited by the power of the motor (F = P/v)."""
        v_base = self.power / self.F_drive_max
        if v <= v_base:
            return self.F_drive_max / self.mass
        else:
            return self.power / (self.mass * v)
        
    def a_grip_max(self, v):
        """Max total tyre acceleration due to available grip (radius of the friction circle)."""
        return self.mew * self.normal_force(v) / self.mass  
    # Speed limit:
    def v_max(self):
        """Top speed (m/s), where the drive force equals drag plus rolling resistance."""
        a = 0.5 * self.rho * (self.CdA + self.Crr * self.ClA)
        b = self.Crr * self.mass * self.g
        roots = np.roots([a, 0, b, -self.power])
        return roots[np.isreal(roots) & (roots.real > 0)].real[0]