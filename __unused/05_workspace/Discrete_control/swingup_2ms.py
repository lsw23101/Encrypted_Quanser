import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import ddt_filter
from pal.utilities.scope import Scope

# ==== Kill switch ====
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# ==== Setup scopes ====
simulationTime = 30
scopePendulum = Scope(title='Pendulum α', timeWindow=10, xLabel='t', yLabel='rad')
scopePendulum.attachSignal(name='α (rad)', width=1)
scopeBase = Scope(title='Base θ', timeWindow=10, xLabel='t', yLabel='rad')
scopeBase.attachSignal(name='θ (rad)', width=1)
scopeVoltage = Scope(title='Motor Voltage', timeWindow=10, xLabel='t', yLabel='V')
scopeVoltage.attachSignal(name='Voltage', width=1)

# ==== Physical parameters ====
Rm = 8.4
kt = 0.042
Mr = 0.095
Lr = 0.085
Mp = 0.024
Lp = 0.129
Jp = Mp * Lp**2 / 12
g = 9.81

Vmax = 20.0
# Swing-up tuning parameters
Er = Mp*g*Lp;  # 목표 에너지 (약 30 mJ)
u_max = 6.0                  # [m/s^2] 최대 가속도
mu = 50.0                    # [m/s/J] 스윙업 게인

# ==== Feedback gain (full state feedback) ====
K = np.array([-1.2247, 24.9044, -0.6877, 3.1321])  # 2ms용 게인

# ==== Utility ====
def wrapToPi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ==== Energy-based swing-up control ====
def swingup_energy(alpha, alpha_dot, mu, u_max, Er, Mr, Lr, Rm, kt, Mp, Lp, Jp, g):
    # 1. 현재 pendulum 에너지
    E = 0.5 * Jp * alpha_dot**2 + Mp * g * (Lp/2) * (1 - np.cos(alpha))
    dE = Er - E
    # 2. 가속도 제어 입력
    u_acc = mu * alpha_dot * np.cos(alpha) * dE
    # 3. Saturation
    u_acc = np.clip(u_acc, -u_max, u_max)
    # 4. 변환: acceleration → torque → voltage
    tau = Mr * Lr * u_acc
    V = (Rm / kt) * tau
    return np.clip(V, -Vmax, Vmax), E

# ==== Main control loop ====
def control_loop():
    qubeVersion = 3
    hardware = 1
    pendulum = 1
    frequency = 500  # 2ms

    state_theta_dot = np.array([0, 0], dtype=np.float64)
    state_alpha_dot = np.array([0, 0], dtype=np.float64)

    if qubeVersion == 2:
        QubeClass = QubeServo2
    else:
        QubeClass = QubeServo3

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:
        start_time = time.time()
        mode = "swingup"
        count = 0
        countMax = frequency // 50

        while (time.time() - start_time) < simulationTime and not KILL_THREAD:
            myQube.read_outputs()
            theta = -myQube.motorPosition
            alpha_raw = myQube.pendulumPosition
            alpha = wrapToPi(alpha_raw - np.pi)

            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)

            x = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # Safety
            if abs(math.degrees(theta)) > 90:
                myQube.write_voltage(0)
                print(">>> SAFETY STOP: base exceeded ±90°")
                break

            if mode == "swingup":
                voltage, E = swingup_energy(alpha, alpha_dot, mu, u_max, Er, Mr, Lr, Rm, kt, Mp, Lp, Jp, g)

                # 전환 조건: pendulum upright 근처
                if abs(math.degrees(alpha)) < 20:
                    mode = "balance"
                    print(">>> Switching to BALANCE mode")

            else:  # balance mode (full state feedback)
                u = float(np.dot(K, x))
                voltage = np.clip(u, -Vmax, Vmax)

            myQube.write_voltage(voltage)

            # Scopes
            count += 1
            if count >= countMax:
                elapsed = time.time() - start_time
                scopePendulum.sample(elapsed, [alpha])
                scopeBase.sample(elapsed, [theta])
                scopeVoltage.sample(elapsed, [voltage])
                count = 0

# ==== Run ====
thread = Thread(target=control_loop)
thread.start()
while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)
input("Press Enter to exit.")
