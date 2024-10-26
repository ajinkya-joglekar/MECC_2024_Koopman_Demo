# Open loop simulation for Open-Loop response in lifted state-space
from F1tenth_utils import lift_states
import numpy as np
def simulate_ol(A,B,C,X,u):
    zt = lift_states.lift_states(X) # Lifting the data in x
    z_prime = np.zeros((zt.shape[0],zt.shape[1])) # Initiate a z_prime matrix
    z_prime[:,0] = zt[:,0] #Initialize value for 0th iteration
    u = u.T
    for i in range(1,u.shape[1]):
            z_prime[:,i] = A@z_prime[:,i-1] + B@u[:,i] # Because X and U are one timestep behind Y, the U considered is ith iteration, also now we consider previous z_prime value
    x_hat_ = C@z_prime # Go back from lifted space to observational space
    return x_hat_