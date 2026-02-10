import socket
import time
import struct
import csv
import ctypes
import os

# ==========================================
# 1. Go 암호화 라이브러리 로드
# ==========================================
lib_name = "./client_crypto.so"
if os.name == 'nt' and not os.path.exists(lib_name):
    lib_name = "./client_crypto.dll"

try:
    lib = ctypes.CDLL(lib_name)
    print(f"[Info] 라이브러리 로드 성공: {lib_name}")
except OSError as e:
    print(f"[Error] 라이브러리를 찾을 수 없습니다. (go build 확인 필요): {e}")
    exit(1)

# --- Go 함수 시그니처 정의 (Generic 버전) ---
lib.InitCrypto.argtypes = [
    ctypes.c_int, ctypes.c_int, ctypes.c_int,         # n, m, p
    ctypes.c_double, ctypes.c_double, ctypes.c_double # s, L, r
]
lib.InitCrypto.restype = None

lib.EncryptVector.argtypes = [
    ctypes.POINTER(ctypes.c_double), # 입력 배열 포인터
    ctypes.c_int,                    # 입력 배열 길이
    ctypes.POINTER(ctypes.c_int)     # 출력 크기 담을 포인터
]
lib.EncryptVector.restype = ctypes.POINTER(ctypes.c_char)

lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype = ctypes.POINTER(ctypes.c_double)

lib.FreePtr.argtypes = [ctypes.c_void_p]


# ==========================================
# 2. Plant Model & Helper (SIMO System)
# ==========================================
N_STATE = 4 # n
N_INPUT = 1 # m (입력 1개)
N_OUTPUT = 2 # p (출력 2개)

# Quantization Parameters
VAL_S = 1.0/100.0
VAL_L = 1.0/100000.0
VAL_R = 1.0/100.0

# System Matrices
A = [
    [1.0000, 0.0301, 0.0200, 0.0002],
    [0.0000, 1.0528, -0.0000, 0.0204],
    [0.0000, 3.0375, 0.9998, 0.0301],
    [0.0000, 5.3236, -0.0002, 1.0528]
]
B = [[0.0100], [0.0099], [1.0043], [1.0001]]
C = [[1.0000, 0.0000, 0.0000, 0.0000], [0.0000, 1.0000, 0.0000, 0.0000]]
xp_ini = [10, 10, 0.0, 0.0]

def mat_vec_mult(M, v):
    return [sum(M[i][j]*v[j] for j in range(len(v))) for i in range(len(M))]

def vec_add(v1, v2):
    return [x + y for x, y in zip(v1, v2)]

# --- 네트워크 헬퍼 함수 ---
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

# --- 암호화 헬퍼 ---
def encrypt_helper(vec):
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
# 3. Main Simulation
# ==========================================
def run_plant_simulation():
    HOST = '127.0.0.1'
    PORT = 8000
    ITERATIONS = 500  # [설정] 반복 횟수 500

    print(f">> Go Crypto Init (n={N_STATE}, m={N_INPUT}, p={N_OUTPUT})...")
    lib.InitCrypto(N_STATE, N_INPUT, N_OUTPUT, VAL_S, VAL_L, VAL_R)

    xp = list(xp_ini)
    data_log = [] 

    print(f"--- Plant Simulation (Inverted Pendulum + FHE) Started ---")
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            while True:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Controller에 연결 성공!")
                    break
                except ConnectionRefusedError:
                    print("..연결 대기중..")
                    time.sleep(1)

            for i in range(ITERATIONS):
                t_loop_start = time.perf_counter()

                # 1. Output y
                y = mat_vec_mult(C, xp)

                # 2. Encrypt & Send y
                y_bytes = encrypt_helper(y)
                send_packet(client_socket, y_bytes)

                # 3. Receive u
                u_enc_bytes = recv_packet(client_socket)
                if not u_enc_bytes: break

                # 4. Decrypt u
                u = decrypt_helper(u_enc_bytes, N_INPUT)
                if u is None: break

                # 5. Re-Encrypt & Send u
                u_reenc_bytes = encrypt_helper(u) 
                send_packet(client_socket, u_reenc_bytes)

                # 6. Update State
                Axp = mat_vec_mult(A, xp)
                Bu = mat_vec_mult(B, u)
                xp = vec_add(Axp, Bu)

                t_loop_total = (time.perf_counter() - t_loop_start) * 1000

                # [변경] Logging: iter, y, u 만 저장 (xp, time 제외)
                log_row = [i] + y + u
                data_log.append(log_row)
                
                if i % 100 == 0:
                    print(f"Iter {i}: y={y} -> u={u} | Loop: {t_loop_total:.2f}ms")

            print("--- Simulation Finished ---")
            
            # CSV 저장
            with open('plant_data_python.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                
                # [변경] Header도 심플하게 수정
                # 결과 예시: iter, y0, y1, u0
                header = ['iter'] + \
                         [f'y{k}' for k in range(N_OUTPUT)] + \
                         [f'u{k}' for k in range(N_INPUT)]
                         
                writer.writerow(header)
                writer.writerows(data_log)
                print(">> saved: plant_data_python.csv (Compact Version)")

    except Exception as e:
        print(f"\n[Error] {e}")

if __name__ == "__main__":
    run_plant_simulation()