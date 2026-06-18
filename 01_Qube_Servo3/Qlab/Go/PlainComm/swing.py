import sys
import os
import time
import struct
import socket
import signal
import math
import numpy as np
from threading import Thread

# 1. Quanser Library Setup
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '00_libraries', 'python'))
from pal.products.qube import QubeServo3
from pal.utilities.scope import Scope
from pal.utilities.math import ddt_filter


# 2. Helper Functions
def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet: return None
        data += packet
    return data

def send_floats(sock, values):
    data = struct.pack(f'<{len(values)}d', *values)
    sock.sendall(struct.pack('<I', len(data)) + data)

def recv_floats(sock, count):
    raw_len = recv_exact(sock, 4)
    if not raw_len: return None
    length = struct.unpack('<I', raw_len)[0]
    data = recv_exact(sock, length)
    if not data: return None
    return list(struct.unpack(f'<{count}d', data))


# Swing-up params
SIGN_THETA = -1.0
SWING_KICK_DIR = 1.0
VMAX = 7.0
mp, Lp, g = 0.024, 0.129, 9.81
l = Lp / 2
Jp = mp * (Lp**2) / 3
mu = 200.0
switch_deg = 15.0

# Full state LQR gain
K_LOCAL = np.array([-2.0, 20.0, -1.5, 2.5])

KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# Plotting
scopePendulum = Scope(title='Pendulum alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopePendulum.attachSignal(name='Alpha', width=1)
scopeBase = Scope(title='Base theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopeBase.attachSignal(name='Theta', width=1)
scopeVoltage = Scope(title='Control Input (V)', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
scopeVoltage.attachSignal(name='u', width=1)


# 3. Main Control Loop
def control_loop():
    global KILL_THREAD

    HOST, PORT = '127.0.0.1', 8000
    N_INPUT = 1
    frequency = 50

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(">> Waiting for Server Connection...")
    while not KILL_THREAD:
        try:
            client_socket.connect((HOST, PORT))
            print(">> Connected to Controller!")
            break
        except ConnectionRefusedError:
            time.sleep(1)

    with QubeServo3(hardware=0, pendulum=1, frequency=frequency) as myQube:
        print(">> Step 1: Swing-up & Local Balancing Started.")

        state_theta_dot = np.array([0, 0], dtype=np.float64)
        state_alpha_dot = np.array([0, 0], dtype=np.float64)

        stable_start_time = None
        is_fully_stable = False
        startTime = time.time()

        # PHASE 1: Swing-up & LQR (Local)
        while not is_fully_stable and not KILL_THREAD:
            myQube.read_outputs()
            theta = myQube.motorPosition * SIGN_THETA
            alpha_raw = myQube.pendulumPosition
            alpha = (alpha_raw - np.pi + np.pi) % (2*np.pi) - np.pi

            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)

            alpha_deg = abs(math.degrees(alpha))
            u = 0.0

            if alpha_deg < switch_deg:
                x_meas = np.array([theta, alpha, theta_dot, alpha_dot])
                u = -1.0 * float(np.dot(K_LOCAL, x_meas))

                if stable_start_time is None:
                    stable_start_time = time.time()
                elif time.time() - stable_start_time > 2.0:
                    is_fully_stable = True
            else:
                stable_start_time = None
                E_total = mp*g*l*(math.cos(alpha)-1) + 0.5*Jp*(alpha_dot**2)
                term = np.sign(alpha_dot * math.cos(alpha))
                u_raw = SWING_KICK_DIR * mu * (0.0 - E_total) * term
                if (theta > 0 and u_raw > 0) or (theta < 0 and u_raw < 0): u = 0.0
                else: u = u_raw

            voltage = np.clip(u, -VMAX, VMAX)
            myQube.write_voltage(-1 * voltage)

            ts = time.time() - startTime
            scopePendulum.sample(ts, [alpha])
            scopeBase.sample(ts, [theta])
            scopeVoltage.sample(ts, [-1 * voltage])

        if KILL_THREAD:
            client_socket.close()
            return

        # ---------------------------------------------------------------------
        # PHASE 2: Encrypted Control (plaintext transport)
        # encrypt.go sends u first; plant reads sensors, writes voltage, sends y.
        # ---------------------------------------------------------------------
        print(">> Step 2: Switching to Encrypted Control.")

        while not KILL_THREAD:
            # 1. u 수신 (Controller → Plant, 먼저 전송)
            u_dec = recv_floats(client_socket, N_INPUT)
            if u_dec is None: break

            # 2. 센서 읽기
            myQube.read_outputs()
            theta = myQube.motorPosition * SIGN_THETA
            alpha = (myQube.pendulumPosition - np.pi + np.pi) % (2*np.pi) - np.pi
            y = [theta, alpha]

            # 3. 구동
            if abs(math.degrees(alpha)) < 20:
                voltage = np.clip(u_dec[0], -VMAX, VMAX)
            else:
                print(">> Out of Range! Emergency Stop.")
                voltage = 0.0
                KILL_THREAD = True
            myQube.write_voltage(voltage)

            # 4. y 전송 (Plant → Controller)
            send_floats(client_socket, y)

            # Plotting
            ts = time.time() - startTime
            scopePendulum.sample(ts, [alpha])
            scopeBase.sample(ts, [theta])
            scopeVoltage.sample(ts, [voltage])

        myQube.write_voltage(0.0)
        client_socket.close()
        print(">> Finished.")


def main():
    global KILL_THREAD
    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()

    while thread.is_alive() and not KILL_THREAD:
        Scope.refreshAll()
        time.sleep(0.01)

    KILL_THREAD = True

if __name__ == "__main__":
    main()
