import socket
import numpy as np, scipy as scp, matplotlib.pyplot as plt, cvxpy as cvx
from pytictoc import TicToc

class PIDController:
    def __init__(self, Kp, Ki, Kd, input_lim, rate_lim, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.input_lim, self.rate_lim = input_lim, rate_lim
        self.previous_error = np.array([0., 0., 0., 0., 0., 0.]).T
        self.integral = np.array([0., 0., 0., 0., 0., 0.]).T
        self.dt = dt

    def input_ratelimiter(self, input_vector):
        change = input_vector - self.previous_error
        for i in range(len(change)):
            if abs(change[i]) > self.rate_lim:
                if change[i] < 0:
                    input_vector[i] = self.previous_error[i] - self.rate_lim
                else:
                    input_vector[i] = self.previous_error[i] + self.rate_lim

    def limit(self, input_vector):
        for i in range(len(input_vector)):
            if abs(input_vector[i]) >= self.input_lim:
                if input_vector[i] < 0:
                    input_vector[i] = -1.0 * self.input_lim
                else:
                    input_vector[i] = self.input_lim  
    

    def u(self, error):
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * self.dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        D_out = self.Kd * derivative
        
        # Compute total output
        pid_output = P_out + I_out + D_out
        self.input_ratelimiter(pid_output)
        self.limit(pid_output)
        
        # Update previous error
        self.previous_error = error
        print(f"hey this is {pid_output}")
        return pid_output[0:3].T

class LQR:
    def __init__(self, A, B, Q, R):
        self.A, self.B = A, B
        self.Q, self.R = Q, R
        P = scp.linalg.solve_discrete_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)

    def u(self, error):
        return self.K @ error


class MPC:
    def __init__(self, A, B):
        self.A, self.B = A, B

    def u(self, initial):
        T = 5
        u = cvx.Variable((3, T))
        s = cvx.Variable((6, T + 1))
        cost = 0
        constraints = []
        for t in range(T):
            cost += cvx.sum_squares(s[:,t + 1]) + cvx.sum_squares(u[:, t])
            constraints += [s[:,t + 1] == self.A @ s[:,t] + self.B @ u[:, t], cvx.norm(u[:, t], "inf") <= 0.5]
        constraints += [s[:, 0] == initial]
        prob = cvx.Problem(cvx.Minimize(cost), constraints)
        prob.solve()
        u = u.value[:,0]
        return -1 * u
        

class KalmanFilter:
    def __init__(self, xk, A, B, H, P, Q, R):
        self.A, self.B = A, B
        self.Q, self.R = Q, R
        self.P = P
        self.H = H
        self.xk = xk

        self.x_list, self.y_list, self.z_list = [], [], []
        self.x_var, self.y_var, self.z_var = [], [], []

    def Prediction(self, uk):
        xk_pred = self.A @ self.xk + self.B @ uk
        P_pred = self.A @ self.P @ self.A.T + self.Q
        return xk_pred, P_pred

    def Posterior(self, xk_pred, zk, P_pred):
        yk = zk - self.H @ xk_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.xk = xk_pred + K @ yk
        self.P = (np.eye(len(self.P)) - K @ self.H) @ P_pred
        
        self.x_list += [self.xk[0]]
        self.y_list += [self.xk[1]]
        self.z_list += [self.xk[2]]
        self.x_var += [self.P[0,0]]
        self.y_var += [self.P[1,1]]
        self.z_var += [self.P[2,2]]
        
    
    def Execute(self, uk, zk):
        xk_pred, P_pred = self.Prediction(uk)
        self.Posterior(xk_pred, zk, P_pred)
    
    
    def graph(self, in_list, out_list, xlabel, ylabel, title, color, marker):
        plt.plot(in_list, out_list, color = color, marker = marker)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True)
        plt.legend()
        plt.show()

    def n_val(self):
        return len(self.x_list)

    def record_xk(self, Time, bin=False):
        self.graph(self.x_list, self.z_list, xlabel='X',ylabel='Z', title='XZ Plane State Estimation', color='blue', marker='o')
        if bin == True:
            self.graph(Time, self.x_list, xlabel='Time (sec)',ylabel='X', title='X State Estimation', color='green', marker='o')
            self.graph(Time, self.z_list, xlabel='Time (sec)',ylabel='Z', title='Z State Estimation', color='purple', marker='o')
    
    def record_var(self, Time):
        self.graph(Time, self.x_var, xlabel='Time (sec)',ylabel='x Variance', title='x Variance Over Time', color='orange', marker='o')
        self.graph(Time, self.z_var, xlabel='Time (sec)',ylabel='z Variance', title='z Variance Over Time', color='orange', marker='o')


class reference_func:

    def __init__(self, ref_x, ref_y, ref_z, dt):
        self.ref_x = ref_x
        self.ref_y = ref_y
        self.ref_z = ref_z
        self.dt = dt
        self.x_vec = []
        self.y_vec = []
        self.z_vec = []
        self.x_in = 0
        self.y_in = 0
        self.z_in = 0

    def Execute(self):
        ref_x = self.ref_x(self.x_in)
        ref_y = self.ref_y(self.y_in)
        ref_z = self.ref_z(self.z_in)
        self.x_in += self.dt
        self.y_in += self.dt
        self.z_in += self.dt
        self.x_vec.append(ref_x)
        self.y_vec.append(ref_y)
        self.z_vec.append(ref_z)

        ref = np.array([ref_x, ref_y, ref_z, 0, 0, 0]).T
        return ref
    
    def plot(self):
        plt.plot(self.x_vec, self.z_vec, color = 'blue', marker = 'o')
        plt.title('Reference Trajectory')
        plt.xlabel('X-Axis')
        plt.ylabel('Z-Axis')
        plt.grid(visible = 'True')
        plt.show()


def Measurement(data, R):
        n = data.size
        zk = np.random.multivariate_normal(data, R).reshape(n,)
        return zk

def error_func(state_estimate, reference):
        return reference - state_estimate 

def Plooter(in_list, out_list, xlabel, ylabel, title, color, marker):
    plt.plot(in_list, out_list, color = color, marker = marker)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()
