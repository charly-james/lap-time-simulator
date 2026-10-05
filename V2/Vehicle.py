from dataclasses import dataclass
import numpy as np

@dataclass
class Vehicle:
    # Constants related to the vehicle's physical properties
    mass: float = 800.0  # kg
    g: float = 9.81      # m/s^2
    power: float = 150e3    # W (peak power at the wheels)
    rho: float = 1.225   # kg/m^3 (air density)
    CdA: float =1.2      # m^2, drag coefficient x frontal area
    ClA: float = 3.0      # m^2, lift (downforce) coefficient x area
    Crr: float = 0.015       # rolling resistance coefficient
    F_drive_max: float = 6400.0  # N, maximum drive force at the wheels (limited by maximum torque and wheel radius)

     # geometry
    a: float = 1.65          # CG to front axle [m]
    b: float = 1.25          # CG to rear axle [m]
    h: float = 0.30          # CG height [m]
    aero_balance: float = 0.42   # fraction of downforce on front axle
    brake_bias: float = 0.58     # fraction of braking force on front axle

    # tyre (Magic Formula, same tyre on both axles)
    mew0: float = 1.7         # peak friction at nominal load
    Fz0: float = 2000.0      # N, nominal load per tyre
    k_load: float = 0.15     # load sensitivity
    By: float = 20.0
    Cy: float = 1.4
    Ey: float = 0.0
    Bx: float = 14.0
    Cx: float = 1.65
    Ex: float = 0.0
   
    @property
    def L(self):
        """Wheelbase."""
        return self.a + self.b

    # Forces:

    def downforce(self, v):
        """Calculate the downforce on the front and rear axles based on the vehicle's speed and aerodynamic balance."""

        total_downforce = 0.5 * self.rho * self.ClA * v**2
        downforce_front = self.aero_balance*total_downforce
        downforce_rear = (1-self.aero_balance)*total_downforce
        return downforce_front, downforce_rear

    def drag(self, v):
        return 0.5 * self.rho * self.CdA * v**2

    def normal_force(self, v, ax):
        """Calculate the normal force on the front and rear axles based on the vehicle's speed, weight distribution, and downforce."""
        [downforce_front, downforce_rear] = self.downforce(v)
        # Front axle:
        Fz_front = (self.mass*self.g*self.b + downforce_front*self.L -self.mass*ax*self.h)/(self.L)
        # Rear axle:
        Fz_rear = (self.mass*self.g*self.a + downforce_rear*self.L + self.mass*ax*self.h)/(self.L)
        return Fz_front, Fz_rear
    

    def rolling_resistance(self, v):
        total_normal_force = self.mass * self.g + sum(self.downforce(v))
        return self.Crr * total_normal_force

    def total_resistance(self, v):
        return self.drag(v) + self.rolling_resistance(v)

    #Friction coefficient:
    
    def mew(self, Fz_axle):
        """Calculate the friction coefficient based on the normal load using a simple linear model."""

        Fz_tyre = Fz_axle / 2.0  # Assuming equal load on both tyres of the axle
        return self.mew0 * (1 - self.k_load * (Fz_tyre - self.Fz0) / self.Fz0)
    
    
    # Acceleration limits:

    def a_power_max(self, v):
        """Calculate the base velocity before which the torque is constant and after which the torque is limited by the power of the motor (F = P/v)."""
        v_base = self.power / self.F_drive_max
        if v <= v_base:
            return self.F_drive_max / self.mass
        else:
            return self.power / (self.mass * v)
        
    def ay_grip_max(self, v):
        """Calculate the maximum lateral acceleration due to available grip."""
        Fz_front, Fz_rear = self.normal_force(v, 0.0)

        # Front axle:Calculate the maximum lateral acceleration the front axle can withstand.
        mew_front = self.mew(Fz_front)
        Fy_max_front =mew_front*Fz_front
        ay_max_front = (self.L*Fy_max_front)/(self.mass*self.b)

        # Rear axle: Calculate the maximum lateral acceleration the rear axle can withstand.
        mew_rear = self.mew(Fz_rear)
        Fy_max_rear = mew_rear*Fz_rear
        ay_max_rear = (self.L*Fy_max_rear)/(self.mass*self.a)
        ay_grip_max = min(ay_max_front, ay_max_rear)
        return ay_grip_max
    

    def ax_grip_max(self, v, ax,kappa):

        """Calculate the maximum longitudinal acceleration due to available grip, considering the lateral acceleration used in corners."""
       
        # Rear axle: Calculate the maximum longitudinal acceleration the rear axle can withstand.

        Fz_front, Fz_rear = self.normal_force(v, ax)
        mew_rear = self.mew(Fz_rear)
        Fy_rear_max = Fx_max = mew_rear*Fz_rear
        ay_used = v**2*abs(kappa)
        Fy_rear_used = min(ay_used*self.a*self.mass/self.L, Fy_rear_max)
        Fx_rear_available = Fx_max*np.sqrt(1-(Fy_rear_used/Fy_rear_max)**2)
        ax_rear_max = Fx_rear_available/self.mass

        #Also, ax reduces the front load, so we need to check that the front tyres can still provide the lateral force needed for cornering.
        # Minimum front load that can still provide Fy_front_used.
        # mew(Fz)*Fz = mew0*(1+k)*Fz - mew0*k*Fz^2/(2*Fz0).
        # In the limit case, mew(Fz)*Fz = Fy_front_used. This gives A*Fz^2 + B*Fz + C = 0.
        Fy_front_used = ay_used*self.b*self.mass/self.L
        if self.k_load == 0:
            # No load sensitivity: mew is constant, so the equation is linear and not quadratic.
            Fz_front_min = Fy_front_used/self.mew0
        else:
            # The equation is quadratic:
            A = self.mew0*self.k_load/(2*self.Fz0)
            B = -self.mew0*(1+self.k_load)
            C = Fy_front_used
            # Smaller root: the larger one is past the peak of mew(Fz)*Fz, outside the range where the linear mew law is valid.
            Fz_front_min = (-B - np.sqrt(B**2 - 4*A*C))/(2*A)  # This is the minimum Fz_front that can still provide the requires lateral force.  

        # Accelerating removes m*ax*h/L from the front, so the max acceleration is the one that brings the front load down to Fz_front_min.
        Fz_front_ax0, _ = self.normal_force(v, 0.0)
        ax_front = (Fz_front_ax0 - Fz_front_min)*self.L/(self.mass*self.h)
        return min(ax_rear_max, ax_front)
    

    def a_braking_max(self, v, ax, kappa):

        """Calculate the maximum braking deceleration due to available grip."""

        # Both axles are being braked so we look at both:
        Fz_front, Fz_rear = self.normal_force(v, ax)
        ay_used = v**2*abs(kappa)

        # Front axle:
        mew_front = self.mew(Fz_front)
        Fy_front_max = Fx_front_max = mew_front*Fz_front
        Fy_front_used = min(ay_used*self.b*self.mass/self.L, Fy_front_max)
        F_braking_front_available = Fx_front_max*np.sqrt(1-(Fy_front_used/Fy_front_max)**2)
        F_braking_max_possible_front = F_braking_front_available/self.brake_bias
        a_braking_max_possible_front = F_braking_max_possible_front/self.mass

        #Rear axle:
        mew_rear = self.mew(Fz_rear)
        Fy_rear_max = Fx_rear_max = mew_rear*Fz_rear
        Fy_rear_used = min(ay_used*self.a*self.mass/self.L, Fy_rear_max)
        F_braking_rear_available = Fx_rear_max*np.sqrt(1-(Fy_rear_used/Fy_rear_max)**2)
        F_braking_max_possible_rear = F_braking_rear_available/(1-self.brake_bias)
        a_braking_max_possible_rear = F_braking_max_possible_rear/self.mass

        # The maximum braking acceleration is the minimum of the two axles' maximum braking accelerations:
        return min(a_braking_max_possible_front, a_braking_max_possible_rear)

        
    def v_max(self):
        """Top speed (m/s), where the drive force equals drag plus rolling resistance."""
        a = 0.5 * self.rho * (self.CdA + self.Crr * self.ClA)
        b = self.Crr * self.mass * self.g
        roots = np.roots([a, 0, b, -self.power])
        return roots[np.isreal(roots) & (roots.real > 0)].real[0]