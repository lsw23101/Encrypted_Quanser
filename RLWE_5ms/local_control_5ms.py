## example balance_control_qube.py
# This example does balance control of the Qube Servo's Pendulum attachment.
# This example uses either a virtual or Physical Qube Servo 2 or Qube Servo 3 device,
# in a task-based (time-based IO) mode where you do not have to handle timing yourself.
# (task based mode is recommended for most applications).

# IF USING HARDWARE, LIFT THE PENDULUM MANUALLY FOR THE CONTROLLER TO KICK IN
# IF USING VIRTUAL, USE THE LIFT PENDULUM BUTTON IN QUANSER INTERACTIVE LABS
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --


# imports

# 퀀서 라이브러리 쓰기 위해서 추가함
import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import SignalGenerator, ddt_filter
from pal.utilities.scope import Scope

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

#################
# 본 실험 장비의 엔코더는 절대 측정이 아님
# 초기 각도를 -180도라고 보고, 초기 위치를 0이라고 봄 >> 0도 위치에서 아래로 내려놓고 시작해야 평형점이 제대로 들어감
##########

# region: Setup
# 시뮬레이션은 30초간 재생 밑에 컬러는 사용 x
simulationTime = 50000 # will run for 30 seconds
color = np.array([0, 1, 0], dtype=np.float64)


scopePendulum = Scope(
    title='Pendulum encoder - alpha (rad)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopePendulum.attachSignal(name='Pendulum - alpha (rad)',  width=1)

scopeBase = Scope(
    title='Base encoder - theta (rad)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopeBase.attachSignal(name='Base - theta (rad)',  width=1)

scopeVoltage = Scope(
    title='Motor Voltage',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Voltage (volts)')
scopeVoltage.attachSignal(name='Voltage',  width=1)

scopeError = Scope(
    title='Control Error (u - voltage)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Error (V)')
scopeError.attachSignal(name='u - voltage', width=1)



# (추가) xhat(옵저버 상태) 모니터링을 위해 스코프에 추정값도 같이 띄우자
scopePendulum.attachSignal(
    name='alpha_hat (rad)',  
    width=1, 
    color=np.array([0, 0, 255])   # 파랑
)
scopeBase.attachSignal(
    name='theta_hat (rad)',      
    width=1, 
    color=np.array([0, 0, 255])   # 파랑
)



#endregion


'''
이산시간 
ds_sys =

'''

# pre-designed controller 
# Design_obsv_LQR_20ms.m 실행 후 복붙


F = np.array([
    [ -0.3491, -0.0302, 0.0061, -0.0017 ],
    [ -0.0069, -0.3701, 0.0010, 0.0033 ],
    [ -89.9690, -9.6081, 1.4239, -0.6779 ],
    [ 0.0793, -99.4450, 0.4191, 0.3311 ]
], dtype=np.float64)

G = np.array([
    [ 1.3530, 0.0119 ],
    [ 0.0107, 1.3535 ],
    [ 91.5104, 2.3047 ],
    [ 1.4449, 92.7937 ]
], dtype=np.float64)

H = np.array([ 6.1957, -32.3594, 1.7040, -2.7325 ], dtype=np.float64)  # shape (4,)



# Code to control the Qube Hardware
# CHANGE qubeVersion, hardware and pendulum VARIABLES FOR DIFFERENT SETUPS
def control_loop():

    # set as 2 or 3 if using a Qube Servo 2 or 3 respectively
    qubeVersion = 3

    # Set as 0 if using virtual Qube Servo
    # Set as 1 if using physical Qube Servo
    hardware = 1

    # Only matters when using virtual Qube. 
    # Set as 0 for virtual DC Motor and 1 for virtual pendulum
    # KEEP AS 1, THIS EXAMPLE USES A PENDULUM
    # not important if using virtual
    pendulum = 1

    '''
    샘플링 시간 맞추기 여기서 설정 해주면 파이썬 루프가 20ms에 알아서 맞춰주는 것으로 추측
    '''
    # 샘플링타임
    frequency = 200  # 5ms

    ################################


    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    # Limit sample rate for scope 
    countMax = frequency / 50
    count = 0

    # full state feedback 상황이며 gain 설계는 2ms 샘플링 타임 기준으로 설계 된 것으로 보임
    # 시스템 모델과 K 게인 설계에 대해서는 튜토리얼이 따로 존재하는 것으로 보이지는 않음
    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-1, 34.75, -1.495, 3.111]) # 사용 x
    else:
        QubeClass = QubeServo3
        # K = np.array([-1.2247, 24.9044, -0.6877, 3.1321]) # 500hz 2ms 에 대한 게인
        # K = np.array([-1.1903, 15.4953, -0.4840, 1.1720]) # 20ms에서 매우 잘되는 게인 
        K = np.array([ 2.0126, -18.0705, 0.6670, -1.4236 ]) # 테스트



    # (추가) 옵저버 상태 초기화
    xhat = np.zeros(4, dtype=np.float64)

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()


        while timeStamp < simulationTime and not KILL_THREAD:
            loopTime = time.time()
            # Read sensor information
            myQube.read_outputs()


            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition
            
            # 출력 값 전처리
            alpha = math.atan2(math.sin(alpha_f - math.pi), math.cos(alpha_f - math.pi))
            alpha_degrees = abs(math.degrees(alpha))  # 임계 판단용 절댓값은 그대로
            # 플랜트 출력 y(t) = [theta, alpha]
            y = np.array([theta, alpha], dtype=np.float64)

            # 실제 state와 비교하기 위한 측정 state 값
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)


            # 레퍼런스 각도 # 안쓰는 중 
            command_deg = 0
            states = command_deg*np.array([np.pi/180, 0, 0, 0]) - np.array([theta, alpha, theta_dot, alpha_dot])
            
            
            # 음성피드백
            ### 제어 입력 연산 
            u = -1.0 * float(H @ xhat) ## 추정 상태로 feedback
            # u = -1.0 * float(H @ x_meas) ## 측정값으로 full state feedback

            VMAX = 10.0 # 제어 입력 staturation

            # 각도가 10도 이내일때만 제어 입력 진행 
            if alpha_degrees > 40:
                voltage = 0
                error = 0  
            else:
                voltage = np.clip(u, -VMAX, VMAX)

            # # Write commands
            # 실제 실험 장비에 제어 입력 전가.
            # myQube.write_voltage(u) 
            myQube.write_voltage(voltage) # voltage는 saturation이 있는 제어 입력
            
            ### 각도가 10도 이내 일때만 state estimation을 해야 처음에 발산하지 않을 것으로 보임 ###

            # 상태 업데이트 (루프 마지막에 추가)
            if alpha_degrees > 40:
                xhat = np.array([0, 0, 0, 0])
                # x_meas = np.array([0, 0, 0, 0])
            else:
                xhat = F @ xhat + G @ y


            # Plot to scopes 각도, 위치, 입력전압 3개가 플랏으로 찍힘
            count += 1
            if count >= countMax:
                # 전압
                scopeVoltage.sample(timeStamp, [voltage])
                # 
                scopeError.sample(timeStamp,    [alpha_degrees])     # ★ 추가

                # 펜듈럼: alpha(측정), alpha_hat(추정)
                scopePendulum.sample(timeStamp, [x_meas[1], xhat[1]])
                # 베이스: theta(측정), theta_hat(추정)
                scopeBase.sample(timeStamp, [x_meas[0], xhat[0]])

                count = 0
            loopTime_end = time.time()
            timeStamp = elapsed_time()
            # if count % 1 == 0:
            #     print("looptime", loopTime_end - loopTime)




# Setup data generation thread and run until complete
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):

    # This must be called regularly or the scope windows will freeze
    # Must be called in the main thread.
    Scope.refreshAll()
    time.sleep(0.01)


input('Press the enter key to exit.')
