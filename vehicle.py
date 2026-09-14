import numpy as np

# Known variables:
mass = 800 #Kg
mew = 1.5
g = 9.81 #m/s^2
a_traction_max = 8.0
a_braking_max = mew * g
a_lateral_max = mew * g
v_max = 40 #m/s
v_limit = []
def corner_speed_limit(kappa):
    kappa_0 = np.where(kappa == 0, 1e-9, kappa)  # avoid divide-by-zero
    v_limit = np.sqrt(a_lateral_max / np.abs(kappa_0))
    v_limit = np.minimum(v_limit, v_max)  # cap at absolute top speed
    return v_limit

def forward_pass(v_limit, ds, a_traction_max, v_0=0.0):
    v_forward = []
    for i in range(len(v_limit)):
        if i == 0:
            v_forward.append(v_0)
        else:
            vel_forward = np.sqrt(v_forward[i-1]**2 + 2 * a_traction_max * ds)
            v_forward.append(min(vel_forward, v_limit[i]))
                
    return np.array(v_forward)

def backward_pass(v_limit, ds, a_braking_max):
    v_backward = []
    for i in range(len(v_limit)-1, -1, -1):
        if i == len(v_limit)-1:
            v_backward.append(v_limit[i])
        else:
            vel_backward = np.sqrt(v_backward[-1]**2 + 2 * a_braking_max * ds)
            v_backward.append(min(vel_backward, v_limit[i]))
    v_backward.reverse()  # reverse the list to match the original order
    return np.array(v_backward)

def forward_pass_ellipse(kappa,v_limit, ds, a_traction_max, a_lateral_max, v_0=0.0):
    v_forward_ellipse = []
    for i in range(len(v_limit)):
        if i == 0:
            v_forward_ellipse.append(v_0)
        else:
            a_lateral_used = v_forward_ellipse[i-1]**2*abs(kappa[i-1])            
            a_long_available = a_traction_max *np.sqrt(1-(a_lateral_used/a_lateral_max)**2 )
            v_possible = np.sqrt(v_forward_ellipse[i-1]**2 + 2 * a_long_available * ds)
            v_forward_ellipse.append(min(v_possible, v_limit[i]))
                
    return np.array(v_forward_ellipse )

def backward_pass_ellipse(kappa, v_limit, ds, a_braking_max, a_lateral_max):
    v_backward_ellipse = []
    for i in range(len(v_limit)-1, -1, -1):
        if i == len(v_limit)-1:
            v_backward_ellipse.append(v_limit[i])
        else:
            a_lateral_used = v_backward_ellipse[-1]**2*abs(kappa[i+1])
            a_long_available = a_braking_max*np.sqrt(1-(a_lateral_used/a_lateral_max)**2 )
            v_possible = np.sqrt(v_backward_ellipse[-1]**2 + 2 * a_long_available * ds)
            v_backward_ellipse.append(min(v_possible, v_limit[i]))
    v_backward_ellipse.reverse()  # reverse the list to match the original order
    return np.array(v_backward_ellipse)

    
def lap_time(v_actual, ds=1.0):
    n_zeros = np.sum(v_actual == 0)
    v_safe = np.where(v_actual == 0, 1, v_actual)
    time_per_step = ds / v_safe
    return np.sum(time_per_step) - ds * n_zeros