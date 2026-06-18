import sys
import os
import time
import struct
import socket
import signal
import math
import numpy as np
from threading import Thread

# ========================================================
# 1. Quanser Library Setup
# ========================================================
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '00_libraries', 'python'))
from pal.products.qube import QubeServo3
from pal.utilities.scope import Scope

# ========================================================
# 2. Helper Functions
# ========================================================
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

# ========================================================
# 3. Scope & Signal Setup
# ========================================================
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

scopePendulum = Scope(title='Pendulum alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopePendulum.attachSignal(name='Alpha', width=1)
scopeBase = Scope(title='Base theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopeBase.attachSignal(name='Theta', width=1)
scopeVoltage = Scope(title='Control Input (V)', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
scopeVoltage.attachSignal(name='u', width=1)

# ========================================================
# 4. Main Control Loop
# ========================================================
def control_loop():
    HOST = '127.0.0.1'
    PORT = 8000
    simulationTime = 60
    N_INPUT, N_OUTPUT = 1, 2
    frequency = 50

    with QubeServo3(hardware=0, pendulum=1, frequency=frequency) as myQube:
        print(">> Hardware Initialized.")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            while not KILL_THREAD:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Connected to Controller!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)

            startTime = time.time()

            while not KILL_THREAD:
                # 1. 센서 읽기
                myQube.read_outputs()
                theta = myQube.motorPosition * -1
                alpha = (myQube.pendulumPosition - np.pi + np.pi) % (2*np.pi) - np.pi
                y = [theta, alpha]

                # 2. y 전송 (Plant → Controller)
                send_floats(client_socket, y)

                # 3. u 수신 (Controller → Plant)
                u = recv_floats(client_socket, N_INPUT)
                if u is None: break

                # 4. 구동 (안전 장치 포함)
                if abs(math.degrees(alpha)) < 15:
                    voltage = np.clip(u[0], -10.0, 10.0)
                else:
                    voltage = 0.0

                myQube.write_voltage(1 * voltage)

                # 5. 로깅 및 플로팅
                timeStamp = time.time() - startTime
                if timeStamp > simulationTime: break
                scopeBase.sample(timeStamp, [theta])
                scopePendulum.sample(timeStamp, [alpha])
                scopeVoltage.sample(timeStamp, [voltage])

            myQube.write_voltage(0.0)
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
