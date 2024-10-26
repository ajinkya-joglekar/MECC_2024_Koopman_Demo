# Import necessary libraries
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import pickle
import numpy as np
import cvxpy as cp
from cvxpy import reshape
from cvxopt import matrix
from tqdm import tqdm
import lift_states

#Koopman MPC function
def Koopman_MPC(X_ref,U_ref):
    # Initialization
    x_ref = X_ref
    z_ref = lift_states.lift_states(x_ref)
    u_ref = U_ref.T
    N = 40  # Prediction horizon

    z_init = np.zeros((z_ref.shape[0], z_ref.shape[1]))
    z_init[:, 0] = z_ref[:, 0]

    u_mpc = np.zeros((u_ref.shape[0], u_ref.shape[1]))

    Q = np.zeros((z_ref.shape[0], z_ref.shape[0]))
    R = np.eye(2)
    Q[1, 1] = 5; Q[2, 2] = 5; Q[3, 3] = 1; Q[4, 4] = 1
    umin = np.array([0, -0.75]).reshape(2)
    umax = np.array([1.5, 0.75]).reshape(2)
    count = 1 # The tqdm loop can be replaced with a while loop if not supported in ROS

    for count in tqdm(range(1, z_ref.shape[1] - N), desc="Loading..."):  # Main loop for solver
        z_pred = cp.Variable((z_ref.shape[0], N + 1))
        u_pred = cp.Variable((u_ref.shape[0], N))
        z_ref_ = matrix(z_ref[:, count:count + N + 1])  # z_ref_ is the reference window for the current iteration of the loop
        cost = 0  # Initializing cost before the loop
        constr = []  # Initializing constaint value

        constr += [z_pred[:, 0] == z_init[:, count - 1]]
        for i in range(N):
            # cost += cp.quad_form(reshape(z_pred[:,i+1],(z_ref.shape[0],1)) - z_ref_[:,i],Q)  # Quad prog required to do (x-x_ref).T@Q@(x-x_ref)
            # constr += [z_pred[:, i + 1] == A_edmd @ z_pred[:, i] + B_edmd @ u_pred[:, i],umin <=u_pred[:,i], u_pred[:,i]<= umax]
            if i == 0:
                cost += cp.quad_form(reshape(z_pred[:, i + 1], (z_ref.shape[0], 1)) - z_ref_[:, i],
                                     Q)  # Quad prog required to do (x-x_ref).T@Q@(x-x_ref)
                constr += [z_pred[:, i + 1] == A_edmd @ z_pred[:, i] + B_edmd @ u_pred[:, i], umin <= u_pred[:, i],
                           u_pred[:, i] <= umax]
            else:
                cost += cp.quad_form(reshape(z_pred[:, i + 1], (z_ref.shape[0], 1)) - z_ref_[:, i], Q) + cp.quad_form(
                    reshape((u_pred[:, i] - u_pred[:, i - 1]) / 0.5, (2, 1)),
                    np.eye(2) * 5)  # Quad prog required to do (x-x_ref).T@Q@(x-x_ref)
                constr += [z_pred[:, i + 1] == A_edmd @ z_pred[:, i] + B_edmd @ u_pred[:, i], umin <= u_pred[:, i],
                           u_pred[:, i] <= umax]

        # constr += [z_pred[:, 0] == z_init[:,count-1]]
        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(solver=cp.OSQP, verbose=False)
        # print(problem.solve())
        u_mpc[:, count - 1] = u_pred.value[:, 0]
        # Publish this U value
        time.pause(0) # Pause needed for control propogation?

        # Read data from
        '''
        z_init[:, count] = A_edmd @ z_init[:, count - 1] + B_edmd @ u_pred.value[:, 0] # keeping this as a reference for how state propogation occurs
        '''
        rate = rospy.Rate(10)


if __name__ == "__main__":
    rospy.init_node('KMPC') # init Koopman node
    '''Optitrack pose topic with callback'''
    mpc_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped,
                                  queue_size=10)
    mpc_msg = AckermannDriveStamped()

    # Load A and B and C EDMD matrix
    EDMD_mat_path = rospy.get_param('EDMD_mat_path')
    A_edmd = np.load(EDMD_mat_path+'A_EDMD_fro.npy')
    B_edmd = np.load(EDMD_mat_path + 'B_EDMD_fro.npy')
    C_edmd = np.load(EDMD_mat_path + 'C_EDMD_fro.npy')

    # Load the data runs and select the trajectory to follow
    data_run_path = rospy.get_param('/data_run') # Path where the runs are stored
    run_name = rospy.get_param('run_name')
    X = np.load(data_run_path+'X_train.npy', allow_pickle=True)
    U = np.load(data_run_path+'U_train.npy', allow_pickle=True)
    with open(data_run_path+'fname','rb') as f:
        run_names = pickle.load(f)
    index = [i for i, x in enumerate(run_names) if x == run_name]
    X_traj = X[index[0]]
    U_traj = U[index[0]]

    Koopman_MPC()

    # rate = rospy.Rate(10)  # Control frequency (Hz)