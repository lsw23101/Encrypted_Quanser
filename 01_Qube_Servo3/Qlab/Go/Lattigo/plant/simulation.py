import socket
import time
import struct
import csv
import ctypes
import json
import os
import numpy as np
import control as ct

_script_dir = os.path.dirname(os.path.abspath(__file__))

# ==========================================
# 1. 파라미터 로드 (offline.go 가 생성한 params.json)
# ==========================================
_params_path = os.path.abspath(os.path.join(_script_dir, '..', 'controller', 'enc_data', 'params.json'))
with open(_params_path) as _f:
    _cfg = json.load(_f)
N_STATE  = _cfg['n']
N_INPUT  = _cfg['m']
N_OUTPUT = _cfg['p']

# ==========================================
# 2. Go 암호화 라이브러리 로드 (스크립트 위치 기준 상대경로)
# ==========================================
if os.name == 'nt':
    _lib_path = os.path.abspath(os.path.join(_script_dir, '..', 'crypto', 'client_crypto.dll'))
else:
    _lib_path = os.path.abspath(os.path.join(_script_dir, '..', 'crypto', 'client_crypto.so'))

try:
    lib = ctypes.CDLL(_lib_path)
    print(f"[Info] 라이브러리 로드 성공: {_lib_path}")
except OSError as e:
    print(f"[Error] 라이브러리를 찾을 수 없습니다. (offline.go 실행 후 go build 확인 필요): {e}")
    exit(1)

lib.InitCrypto.argtypes = [ctypes.c_char_p]
lib.InitCrypto.restype = None

lib.EncryptVector.argtypes = [
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int)
]
lib.EncryptVector.restype = ctypes.POINTER(ctypes.c_char)

lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype = ctypes.POINTER(ctypes.c_double)

lib.FreePtr.argtypes = [ctypes.c_void_p]


# ==========================================
# 2. Plant Model Class (Rotary Pendulum)
# ==========================================
class rotpen:
    def __init__(self, ts):
        # Continuous time linear model matrices
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
        D_c = np.array([[0],
                        [0]], dtype=float)
        
        # Create State Space system and discretize (c2d)
        sys_c = ct.ss(A_c, B_c, C_c, D_c)
        sys_d = sys_c.sample(ts, method='zoh')
        
        # Save discretized matrices
        self.A = sys_d.A
        self.B = sys_d.B
        self.C = C_c  # Output matrix C usually remains same unless direct feedthrough changes
        self.D = D_c

        # Initialize state
        self.xp = np.zeros((4,1), dtype=float)

    def set_init(self, x_init_list):
        # x_init_list: list or 1D array of length 4
        self.xp = np.array(x_init_list, dtype=float).reshape(4, 1)

    def state_update(self, u_list):
        # u_list: list or 1D array of input
        u = np.array(u_list, dtype=float).reshape(-1, 1)
        
        # Update state: x[k+1] = A x[k] + B u[k]
        self.xp = self.A @ self.xp + self.B @ u

    def get_output(self):
        # y[k] = C x[k]
        y = self.C @ self.xp
        # Convert numpy array back to standard python list for compatibility with crypto helper
        return y.flatten().tolist()


# ==========================================
# 3. Network & Crypto Helpers
# ==========================================
def send_packet(sock, data_bytes):
    length = len(data_bytes)
    sock.sendall(struct.pack('<I', length) + data_bytes)

def recv_packet(sock):
    raw_len = recv_exact(sock, 4)
    if not raw_len: return None
    length = struct.unpack('<I', raw_len)[0]
    return recv_exact(sock, length)

def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet: return None
        data += packet
    return data

def encrypt_helper(vec):
    # vec must be a list of floats
    c_arr = (ctypes.c_double * len(vec))(*vec)
    size = ctypes.c_int()
    ptr = lib.EncryptVector(c_arr, len(vec), ctypes.byref(size))
    data = ctypes.string_at(ptr, size.value)
    lib.FreePtr(ptr)
    return data

def decrypt_helper(data_bytes, out_len):
    ptr = lib.DecryptVector(data_bytes, len(data_bytes), out_len)
    if not ptr: return None
    res = [ptr[i] for i in range(out_len)]
    lib.FreePtr(ptr)
    return res


# ==========================================
# 4. Main Simulation
# ==========================================
def run_plant_simulation():
    # Init Crypto (파라미터는 params.json 에서 자동 로드)
    print(f">> Go Crypto Init (n={N_STATE}, m={N_INPUT}, p={N_OUTPUT})...")
    lib.InitCrypto(_params_path.encode())

    # Initialize Plant
    TS = 0.02 # Sampling Time (예시: 0.02초, 필요에 따라 조정)
    plant = rotpen(TS)
    
    # Initial State [0, 0.5, 0, 0]
    xp_ini = [0, 0.5, 0.0, 0.0]
    plant.set_init(xp_ini)
    
    data_log = [] 
    
    HOST = '127.0.0.1'
    PORT = 8000
    ITERATIONS = 500

    print(f"--- Plant Simulation (Optimized Flow / Control Lib) Started ---")
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            # Connection Retry Loop
            while True:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Controller에 연결 성공!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)

            for i in range(ITERATIONS):
                t_loop_start = time.perf_counter()

                # -----------------------------------------------------
                # 1. Output y 측정 (현재 상태 기준)
                # -----------------------------------------------------
                y = plant.get_output()

                # -----------------------------------------------------
                # 2. Receive u (Controller가 먼저 보냄)
                # -----------------------------------------------------
                u_enc_bytes = recv_packet(client_socket)
                if not u_enc_bytes: break

                # -----------------------------------------------------
                # 3. Decrypt u
                # -----------------------------------------------------
                u = decrypt_helper(u_enc_bytes, N_INPUT) # Returns list [u1]
                if u is None: break
                print(i, u)
                # -----------------------------------------------------
                # 4. Pack [y | u] -> Encrypt -> Send
                # -----------------------------------------------------
                combined_vec = y + u 
                combined_bytes = encrypt_helper(combined_vec)


                # send
                send_packet(client_socket, combined_bytes)

                # -----------------------------------------------------
                # 5. Plant State Update
                # -----------------------------------------------------
                plant.state_update(u)

                t_loop_total = (time.perf_counter() - t_loop_start) * 1000

                # Logging
                log_row = [i] + y + u
                data_log.append(log_row)
                
                if i % 100 == 0:
                    print(f"Iter {i}: y={[round(v,4) for v in y]} u={[round(v,4) for v in u]} | Loop: {t_loop_total:.2f}ms")

            print("--- Simulation Finished ---")
            
            # CSV save
            with open('sim_data/plant_data_python.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                header = ['iter'] + [f'y{k}' for k in range(N_OUTPUT)] + [f'u{k}' for k in range(N_INPUT)]
                writer.writerow(header)
                writer.writerows(data_log)
                print(">> saved: plant_data_python.csv")

    except Exception as e:
        print(f"\n[Error] {e}")

if __name__ == "__main__":
    run_plant_simulation()