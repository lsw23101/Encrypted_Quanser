import socket
import time
import struct
import csv
import numpy as np
import control as ct


# ==========================================
# 1. Plant Model Class (Rotary Pendulum)
# ==========================================
class rotpen:
    def __init__(self, ts):
        A_c = np.array([[0, 0, 1, 0],
                        [0, 0, 0, 1],
                        [0, 149.275096865093, -0.0104427822555581, 0],
                        [0, 261.609107366662, -0.0103213545549121, 0]], dtype=float)
        B_c = np.array([[0],
                        [0],
                        [49.7275345502766],
                        [49.1493074043432]], dtype=float)
        C_c = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0]], dtype=float)
        D_c = np.array([[0], [0]], dtype=float)

        sys_c = ct.ss(A_c, B_c, C_c, D_c)
        sys_d = sys_c.sample(ts, method='zoh')

        self.A = sys_d.A
        self.B = sys_d.B
        self.C = C_c
        self.xp = np.zeros((4, 1), dtype=float)

    def set_init(self, x_init_list):
        self.xp = np.array(x_init_list, dtype=float).reshape(4, 1)

    def get_output(self):
        return (self.C @ self.xp).flatten().tolist()

    def state_update(self, u_list):
        u = np.array(u_list, dtype=float).reshape(-1, 1)
        self.xp = self.A @ self.xp + self.B @ u


# ==========================================
# 2. Network Helpers
# ==========================================
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


# ==========================================
# 3. Main Simulation
# ==========================================
def run_plant_simulation():
    N_INPUT = 1
    N_OUTPUT = 2

    TS = 0.02
    plant = rotpen(TS)
    plant.set_init([0, 0.5, 0.0, 0.0])

    data_log = []
    HOST = '127.0.0.1'
    PORT = 8000
    ITERATIONS = 500

    print("--- Plant Simulation (Plaintext) Started ---")

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            while True:
                try:
                    sock.connect((HOST, PORT))
                    print(">> Controller에 연결 성공!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)

            for i in range(ITERATIONS):
                t_start = time.perf_counter()

                # 1. y 측정
                y = plant.get_output()

                # 2. y 전송 (Plant → Controller)
                send_floats(sock, y)

                # 3. u 수신 (Controller → Plant)
                u = recv_floats(sock, N_INPUT)
                if u is None: break

                # 4. 플랜트 상태 업데이트
                plant.state_update(u)

                t_total = (time.perf_counter() - t_start) * 1000
                data_log.append([i] + y + u)

                if i % 100 == 0:
                    print(f"Iter {i}: y={[round(v,4) for v in y]} u={[round(v,4) for v in u]} | Loop: {t_total:.2f}ms")

            print("--- Simulation Finished ---")

            with open('sim_data/plant_data_python.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['iter'] + [f'y{k}' for k in range(N_OUTPUT)] + [f'u{k}' for k in range(N_INPUT)])
                writer.writerows(data_log)
                print(">> saved: plant_data_python.csv")

    except Exception as e:
        print(f"\n[Error] {e}")

if __name__ == "__main__":
    run_plant_simulation()
