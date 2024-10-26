import numpy as np
from F1tenth_utils import lift_states

def simulate_ol(A,B,z,u):
    # zt = lift_states.lift_states(X) # Lifting the data in x

    z_prime = np.zeros((z.shape[0],z.shape[1]))
    # print(z_prime.shape,u.shape,A.shape,B.shape)
    # z_prime[:,0] = zt[:,0]
    # print(A.shape,B.shape,X.shape,u.shape)
    z_prime[:,0] = A@z[:,0] + B@u[:,0] #This is like the first value of Y array with 1st value of X and U
    for i in range(1,u.shape[1]):
            z_prime[:,i] = A@z_prime[:,i-1] + B@u[:,i] # Because X and U are one timestep behind Y, the U considered is ith iteration, also now we consider previous z_prime value
    return z_prime