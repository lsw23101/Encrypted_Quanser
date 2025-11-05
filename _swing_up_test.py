# -*- coding: utf-8 -*-
# example balance_control_qube_with_swingup_reftracking.py
# 원본 구조 유지 + 에너지 스윙업(아래 기준) + θ_ref 추종 + 자동 전환

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

# ========== kill thread ==========
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

#################
# 초기: 펜듈럼 아래로 두고 시작 (down = 0 rad)
#################

# ========== region: Setup ==========
simulationTime = 50000
color = np.array([0, 1, 0], dtype=np.float64)

scopePendulum = Scope(title='Pendulum - alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='rad')
scopePendulum.attachSignal(name='alpha_meas (rad)', width=1)

scopeBase = Scope(title='Base - theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='rad')
scopeBase.attachSignal(name='theta_meas (rad)', width=1)

scopeVoltage = Scope(title='Motor Voltage', timeWindow=10, xLabel='Time (s)', yLabel='V')
scopeVoltage.attachSignal(name='Voltage', width=1)

scopeMon = Scope(title='Monitor', timeWindow=10, xLabel='Time (s)', yLabel='value')
scopeMon.attachSignal(name='|alpha_up| (deg)', width=1)
scopeMon.attachSignal(name='E - E*', width=1)

# xhat 시각화
scopePendulum.attachSignal(name='alpha_hat (rad)', width=1, color=np.array([0, 0, 255]))
scopeBase.attachSignal(name='theta_hat (rad)',  width=1, color=np.array([0, 0, 255]))
# ========== endregion ==========


'''
이산 20ms 모델 (원본 주석 유지)
'''

# pre-designed observer/controller gains (원본 유지)
F = np.array([
    [-0.2886, -0.2902,  0.0318, -0.0202],
    [ 0.0393, -0.6374,  0.0117,  0.0001],
    [-17.9294,-27.1040, 2.1893, -2.0234],
    [ 4.1026, -50.3645, 1.1844, -0.9922]
], dtype=np.float64)

G = np.array([
    [ 1.3303,  0.0732],
    [ 0.0021,  1.4450],
    [22.1180,  5.3075],
    [ 0.0685, 30.9571]
], dtype=np.float64)

H = np.array([ 4.1707, -24.7282, 1.1845, -2.0448 ], dtype=np.float64)

# ===== 물리 파라미터 (Qube3 근사) =====
g   = 9.81
Mp  = 0.024        # kg
Lp  = 0.129        # m
lc  = Lp/2.0       # m
Jp  = (1.0/3.0)*Mp*Lp**2  # pendulum inertia about pivot (uniform rod)

# ===== 스윙업 (에너지/참조추종) 파라미터 =====
Ke      = 1.8      # 에너지→theta_ref 스케일 (너무 크면 난폭)
theta_ref_max = 0.6    # rad, 베이스 목표 최대각 (약 34°)
Kp_th   = 25.0     # 베이스 PD (θ) 비례
Kd_th   = 2.5      # 베이스 PD (θ) 미분
VMAX    = 10.0     # 전압 포화

# ===== 밸런스 진입 임계 (직립 기준) =====
alpha_catch_deg = 15.0  # |alpha_up| < 15°
alphadot_catch  = 2.5   # |alpha_dot| < 2.5 rad/s
hysteresis_deg  = 3.0

# ===== 밸런스 모드에서 옵저버 사용 =====
use_observer_in_balance = True   # True 권장

def wrap_pi(x):
    return math.atan2(math.sin(x), math.cos(x))

def control_loop():
    qubeVersion = 3
    hardware = 1    # 0: virtual, 1: physical
    pendulum = 1

    frequency = 50  # 20ms
    Ts = 1.0 / frequency

    state_theta_dot = np.array([0, 0], dtype=np.float64)
    state_alpha_dot_down = np.array([0, 0], dtype=np.float64)
    state_alpha_dot_up   = np.array([0, 0], dtype=np.float64)

    countMax = frequency / 50
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
    else:
        QubeClass = QubeServo3

    xhat = np.zeros(4, dtype=np.float64)
    MODE = "SWINGUP"

    # 에너지 목표 (down 기준에서 upright까지 필요 에너지)
    E_star = Mp * g * lc * 2.0

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:
        startTime = time.time()
        timeStamp = 0.0

        while timeStamp < simulationTime and not KILL_THREAD:
            t0 = time.time()

            myQube.read_outputs()

            # ===== 측정 =====
            theta    = myQube.motorPosition * -1.0   # 원래 코드 부호 유지
            alpha_f  = myQube.pendulumPosition

            # 아래 기준 각도 (down=0 rad) → 에너지 계산용
            alpha_down = wrap_pi(alpha_f)  # down=0
            # 직립 기준 각도 (up=0 rad) → 스위칭 판단용
            alpha_up   = wrap_pi(alpha_f - math.pi)  # up=0

            # 도함수 추정
            theta_dot, state_theta_dot         = ddt_filter(theta,      state_theta_dot,      50, Ts)
            alpha_dot_down, state_alpha_dot_down = ddt_filter(alpha_down, state_alpha_dot_down, 100, Ts)
            alpha_dot_up,   state_alpha_dot_up   = ddt_filter(alpha_up,   state_alpha_dot_up,   100, Ts)

            # 에너지 (down 기준)
            E = 0.5 * Jp * (alpha_dot_down**2) + Mp * g * lc * (1.0 - math.cos(alpha_down))
            E_err = E - E_star

            # ===== 모드 전환 =====
            abs_alpha_up_deg = abs(math.degrees(alpha_up))
            if MODE == "SWINGUP":
                if (abs_alpha_up_deg < (alpha_catch_deg - hysteresis_deg)) and (abs(alpha_dot_up) < alphadot_catch):
                    MODE = "BALANCE"
            else:
                if abs_alpha_up_deg > (alpha_catch_deg + hysteresis_deg):
                    MODE = "SWINGUP"

            # ===== 제어 =====
            if MODE == "SWINGUP":
                # 에너지 오차로 목표 theta_ref 생성 (부호: dot(alpha_down)*cos(alpha_down))
                sign_term = 0.0
                c = math.cos(alpha_down)
                if abs(alpha_dot_down) > 1e-4 and abs(c) > 1e-3:
                    sign_term = math.copysign(1.0, alpha_dot_down * c)

                theta_ref = Ke * E_err * sign_term
                # 과도 방지: 클램프
                theta_ref = max(-theta_ref_max, min(theta_ref_max, theta_ref))

                # 베이스 PD 추종
                u = Kp_th * (theta_ref - theta) - Kd_th * theta_dot

                # 보호: α가 너무 멀면(거의 수평 이상) 잠시 전압 0으로 진정
                if abs_alpha_up_deg > 170.0:
                    u = 0.0

                voltage = float(np.clip(u, -VMAX, VMAX))

                # 옵저버는 스윙업 동안엔 “느슨히” 업데이트(발산 방지)
                if abs_alpha_up_deg < 120.0:
                    xhat = F @ xhat + G @ np.array([theta, alpha_up], dtype=np.float64)
                else:
                    xhat = np.zeros(4, dtype=np.float64)

            else:  # BALANCE
                # 밸런스: 옵저버 상태 피드백(LQR)
                xhat = F @ xhat + G @ np.array([theta, alpha_up], dtype=np.float64)
                if use_observer_in_balance:
                    u = -1.0 * float(H @ xhat)
                else:
                    # 혹시 옵저버 없이 측정 기반으로 쓰고 싶다면(비추천):
                    x_meas = np.array([theta, alpha_up, theta_dot, alpha_dot_up], dtype=np.float64)
                    u = -1.0 * float(H @ x_meas)
                voltage = float(np.clip(u, -VMAX, VMAX))

            # ===== 전압 인가 =====
            myQube.write_voltage(voltage)

            # ===== 스코프 =====
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [voltage])
                scopePendulum.sample(timeStamp, [alpha_up, xhat[1]])   # up 기준과 xhat 비교
                scopeBase.sample(timeStamp, [theta, xhat[0]])
                scopeMon.sample(timeStamp, [abs_alpha_up_deg, E_err])
                count = 0

            # ===== 루프 타이밍 =====
            timeStamp = time.time() - startTime
            print("mode:", MODE, " theta_ref:" if MODE=="SWINGUP" else "", (theta_ref if MODE=="SWINGUP" else ""), " u:", voltage)

            # (옵션) 고정 주기 보정이 필요하면 여기서 sleep 조절
            # dt = time.time() - t0
            # if dt < Ts:
            #     time.sleep(Ts - dt)


thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
