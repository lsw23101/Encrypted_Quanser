import socket
import struct
import io
import time
import numpy as np

# ==========================================
# Controller Matrices
# ==========================================
F = np.array([
    [ 0.6951, -0.8117,  0.0756, -0.0696],
    [ 0.2336, -0.3680,  0.0552, -0.0489],
    [21.0411, -78.7214, 6.5861, -6.9828],
    [24.5716, -82.6764, 5.5629, -5.9310]
], dtype=np.float64)

G = np.array([
    [0.5524,  0.0679],
    [0.0119,  0.6530],
    [3.8233,  3.9944],
    [0.1897, 10.5578]
], dtype=np.float64)

H = np.array([24.7585, -77.4332, 5.5625, -6.9830], dtype=np.float64)


# ==========================================
# Network Helpers
# ==========================================
def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def send_floats(sock, values):
    data = struct.pack(f'<{len(values)}d', *values)
    sock.sendall(struct.pack('<I', len(data)) + data)

def recv_floats(sock, count):
    raw_len = recv_exact(sock, 4)
    if not raw_len:
        return None
    length = struct.unpack('<I', raw_len)[0]
    data = recv_exact(sock, length)
    if not data:
        return None
    return list(struct.unpack(f'<{count}d', data))


# ==========================================
# Controller Loop
# ==========================================
def handle_plant(conn, addr):
    print(f">> Plant Connected: {addr}")
    n_state = 4
    n_output = 2
    n_input = 1

    x = np.zeros(n_state, dtype=np.float64)
    iter_count = 0

    try:
        while True:
            start = time.perf_counter()

            # 1. y 수신 (Plant → Controller)
            y = recv_floats(conn, n_output)
            if y is None:
                break
            y = np.array(y, dtype=np.float64)

            # 2. u = H @ x 계산 후 전송
            u = float(H @ x)
            send_floats(conn, [u])

            # 3. 상태 업데이트: x = F @ x + G @ y
            x = F @ x + G @ y

            elapsed = (time.perf_counter() - start) * 1000
            if iter_count % 100 == 0:
                print(f"Iter {iter_count}: Cycle Time {elapsed:.2f} ms")
            iter_count += 1

    except Exception as e:
        print(f"[Error] {e}")
    finally:
        conn.close()
        print(">> Plant Disconnected.")


def main():
    HOST, PORT = 'localhost', 8000

    print("--- Plaintext Controller (Python) ---")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen()
        print(f"Waiting for Plant at {HOST}:{PORT}...")

        while True:
            conn, addr = server.accept()
            handle_plant(conn, addr)


if __name__ == "__main__":
    main()
