import numpy as np

def lift_states(X):
    # np.seterr(invalid='ignore')
    z_lift = np.zeros([13,X.shape[0]])
    for i in range(X.shape[0]):
        data =  X[i,:]
        x1 = data[0]
        x2 = data[1]
        x3 = data[2]
        x4 = data[3]
        t2 = np.cos(x4)
        t3 = np.sin(x4)
        # t4 = x3**2
        # t5 = x3**3
        # t7 = x3**5
        # t6 = t4**2
        t8 = np.multiply(t2,x3)
        t9 = np.multiply(t3,x3)
        # t10 = np.multiply(t2,t4)
        # t11 = np.multiply(t2,t5)
        # t13 = np.multiply(t2,t7)
        # t14 = np.multiply(t3,t4)
        # t15 = np.multiply(t3,t5)
        # t17 = np.multiply(t3,t7)
        # t12 = np.multiply(t2,t6)
        # t16 = np.multiply(t3,t6)
        # t17 = np.arctan2(x2,x1)
        t18 = np.multiply(t2,x2)
        t19 = np.multiply(t2, x1)
        t20 = np.multiply(t3,x2)
        t21 = np.multiply(t3, x1)
        D = np.array([1.0,x1,x2,x3,x4,t2,t3,t8,t9,t18,t19,t20,t21]).T
        # print(D.shape)
        z_lift[:,i] = D
        # z_lift.reshape(17,-1)
    # z_lift[np.isnan(z_lift)] = 0
    return z_lift
#%%
