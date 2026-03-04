## example balance_control_qube.py
# This example does balance control of the Qube Servo's Pendulum attachment.
# This example uses either a virtual or Physical Qube Servo 2 or Qube Servo 3 device,
# in a task-based (time-based IO) mode where you do not have to handle timing yourself.
# (task based mode is recommended for most applications).

# IF USING HARDWARE, LIFT THE PENDULUM MANUALLY FOR THE CONTROLLER TO KICK IN
# IF USING VIRTUAL, USE THE LIFT PENDULUM BUTTON IN QUANSER INTERACTIVE LABS
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --


'''
현재 50ms 샘플링 타임으로 full state feedback control 가능 (Design_LQR.m 파일에서 게인 구하기 가능)

TODO:
1. C 행렬 수정하여 observer based controller 설계
>> observe가 잘 되는지 확인

2. x = [몸통각도, 진자각도, 몸통각속도, 진자각속도]  = [theta alpha theta_dot alpha_dot]

3. state observate가 잘 안된다...

'''




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
simulationTime = 30 # will run for 30 seconds
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
이산시간 20ms 모델링
ds_sys =
 
  A = 
               x1          x2          x3          x4
   x1           1       0.197     0.04999    0.003213
   x2           0       1.345  -1.359e-05     0.05563
   x3           0       8.302      0.9994       0.197
   x4           0       14.55  -0.0005729       1.345
 
  B = 
            u1
   x1   0.0641
   x2  0.06485
   x3    2.644
   x4    2.734
 
  C = 
       x1  x2  x3  x4
   y1   1   0   0   0
   y2   0   1   0   0

 
  D = 
       u1
   y1   0
   y2   0


'''



F = np.array([
    [ 0.6951, -0.8117, 0.0756, -0.0696 ],
    [ 0.2336, -0.3680, 0.0552, -0.0489 ],
    [ 21.0411, -78.7214, 6.5861, -6.9828 ],
    [ 24.5716, -82.6764, 5.5629, -5.9310 ]
], dtype=np.float64)

G = np.array([
    [ 0.5524, 0.0679 ],
    [ 0.0119, 0.6530 ],
    [ 3.8233, 3.9944 ],
    [ 0.1897, 10.5578 ]
], dtype=np.float64)

H = np.array([ 24.7585, -77.4332, 5.5625, -6.9830 ], dtype=np.float64)  # shape (4,)





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
    샘플링 시간 맞추기
    '''
    # 샘플링타임
    frequency = 50  # 20ms
    # frequency = 20 # 20Hz가 50ms 

    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    # Limit sample rate for scope 
    countMax = frequency / 50
    count = 0

    # full state feedback 상황이며 gain 설계는 2ms 샘플링 타임 기준으로 설계 된 것으로 보임
    # 시스템 모델과 K 게인 설계에 대해서는 튜토리얼이 따로 존재하는 것으로 보이지는 않음
    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-1, 34.75, -1.495, 3.111])
    else:
        QubeClass = QubeServo3
        # K = np.array([-1.2247, 24.9044, -0.6877, 3.1321]) 2ms 에 대한 게인
        K = np.array([-1.1903, 15.4953, -0.4840, 1.1720])

    # (추가) 옵저버 상태 초기화
    xhat = np.zeros(4, dtype=np.float64)




    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        while timeStamp < simulationTime and not KILL_THREAD:

            # Read sensor information
            myQube.read_outputs()

            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition
            
            # ★ 업라이트(위쪽)가 0도가 되도록 시프트
            alpha = (alpha_f - np.pi + np.pi) % (2*np.pi) - np.pi       # alpha는 rad 단위
            alpha_degrees = abs(math.degrees(alpha))                    # 밑에 if문을 위해서 그냥 degree로 변환...

            # 여기서 y가 크기 2짜리 센서 출력 값
            y = np.array([theta, alpha], dtype=np.float64)


            # Calculate angular velocities with filter of 50 and 100 rad
            # 미분필터 50이랑 100은 bandwidth // ddt_filter 함수에서 알아서 처리...?
            # 따라서 x와 theta 직접 측정한 이후 이 값을 직접 미분하여 full state를 측정 가능한 것으로 주어짐
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)


            # 레퍼런스 각도 # 안쓰는 중 
            command_deg = 0
            states = command_deg*np.array([np.pi/180, 0, 0, 0]) - np.array([theta, alpha, theta_dot, alpha_dot])
            
            
            # 음성피드백을 감안해줘야하는거같음 -1을 곱해주어야 작동
            u = -1 * float(H @ xhat)
            # u = float(H @ xhat)
            VMAX = 10.0


            # 각도가 10도 이내일때만 제어 입력 진행 u = -K x 형태의 full state feedback 제어
            if alpha_degrees > 10:
                voltage = 0
                error = 0
                
            else:
                # voltage = np.dot(K, xhat)
                
                voltage = np.clip(u, -VMAX, VMAX)

                # 일단은 옵저버 설계를 위해 측정 값으로 control
                # 음성피드백을 감안해줘야하는거같음 -1을 곱해주어야 작동
                # voltage = -1 * np.dot(H, x_meas)          # 스칼라
                # error = u - voltage


            # # Write commands
            # 실제 실험 장비에 전압 인가하는 함수로 보임
            myQube.write_voltage(u)
            # myQube.write_voltage(voltage)
            

            # 각도가 10도 이내 일때만 state estimation을 해야 처음에 발산하지 않을 것으로 보임
            if alpha_degrees > 5:
                xhat = np.array([0, 0, 0, 0])
            else:
                xhat = F @ xhat + G @ y


            # Plot to scopes 각도, 위치, 입력전압 3개가 플랏으로 찍힘
            count += 1
            if count >= countMax:
                # 전압
                scopeVoltage.sample(timeStamp, [voltage])
                # 옵저버 베이스드 컨트롤러의 제어입력과 full state의 제어 입력 차이
                scopeError.sample(timeStamp,    [u])     # ★ 추가

                # 펜듈럼: alpha(측정), alpha_hat(추정)
                scopePendulum.sample(timeStamp, [alpha, xhat[1]])
                # 베이스: theta(측정), theta_hat(추정)
                scopeBase.sample(timeStamp, [theta, xhat[0]])

                count = 0

            timeStamp = elapsed_time()



# Setup data generation thread and run until complete
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):

    # This must be called regularly or the scope windows will freeze
    # Must be called in the main thread.
    Scope.refreshAll()
    time.sleep(0.01)


input('Press the enter key to exit.')
