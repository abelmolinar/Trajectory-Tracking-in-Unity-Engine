from Unity_Classes import *

def ref_x(dt):
    return -15

def ref_y(dt):
    return 0.

def ref_z(dt):
    return 20

dt = 0.1
Reference = reference_func(ref_x, ref_y, ref_z, dt = 1/60)
control_vector = np.array([0, 0, 0]).T

process_var, meas_var = 0.1, 0.1

xk = np.array([0, 0, 0, 0, 0, 0]).T

A = np.array([[1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]])

B = np.array([[0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]])

H = np.eye(6)
            
P = np.eye(6) * meas_var

Q = np.eye(6) * process_var
R = np.eye(6) * meas_var

State_Est = KalmanFilter(xk, A, B, H, P, Q, R)


# Controller initialization

# PID
Kp, Ki, Kd = 10., 0.01, 50. ; input_lim, rate_lim = 0.5, 2. ; Control = PIDController(Kp, Ki, Kd, input_lim, rate_lim, dt)

# LQR
# SM = 10 * np.eye(6); IM = 1000 * np.eye(3); Control = LQR(A, B, SM, IM)

# MPC
# Control = MPC(A, B)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('127.0.0.1', 5000)
print(f"Starting server on {server_address[0]}:{server_address[1]}")
server_socket.bind(server_address)
server_socket.listen(1)

print("Waiting for connection...")
connection, client_address = server_socket.accept()
print(f"Connected to {client_address}")

xp, zp = [], []
t = TicToc()
t.tic()
try:
    while True:
         # Receive current position from Unity
        data = connection.recv(1024).decode().strip()
        current_position = list(map(float, data.split(',')))  # Parse x, y, z
        print(f"Received current position: {current_position}")
        current_position = np.array(current_position)

        # Measurement Function
        zk = Measurement(current_position, R)
        reference = Reference.Execute()

        # Develop a state estimation
        State_Est.Execute(control_vector, zk)
        xk_StateEst = State_Est.xk

        # Calculate the error
        error = error_func(xk_StateEst, reference)
        # error = error_func(current_position, reference)

        # Controller
        control_vector = Control.u(error)


        # x and z positions are saved
        xp.append(current_position[0])
        zp.append(current_position[2])

        
    
        # Send control inputs back to Unity
        control_data = ','.join(map(str, control_vector))
        connection.sendall(control_data.encode())
        print(f"Sent control inputs: {control_data}")

finally:
    t.toc()
    T_final = t.tocvalue()
    connection.close()
    server_socket.close()
    time_vec = np.linspace(start=0, stop=T_final, num=State_Est.n_val())
    State_Est.record_xk(time_vec, bin=True)
    # State_Est.record_var(time_vec)
    # Reference.plot()
    # Plooter(xp, zp, 'X', 'Z', '2D Plane', color='red', marker='o')
