## example balance_control_qube.py
# This example does balance control of the Qube Servo's Pendulum attachment.
# This example uses either a virtual or Physical Qube Servo 2 or Qube Servo 3 device,
# in a task-based (time-based IO) mode where you do not have to handle timing yourself.
# (task based mode is recommended for most applications).

# IF USING HARDWARE, LIFT THE PENDULUM MANUALLY FOR THE CONTROLLER TO KICK IN
# IF USING VIRTUAL, USE THE LIFT PENDULUM BUTTON IN QUANSER INTERACTIVE LABS
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --



# imports

# Quanser libraries
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '..', '00_libraries', 'python'))

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


# simulation parameters
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


scopePendulum.attachSignal(
    name='alpha_hat (rad)',  
    width=1, 
    color=np.array([0, 0, 255])   
)
scopeBase.attachSignal(
    name='theta_hat (rad)',      
    width=1, 
    color=np.array([0, 0, 255])   
)



# Pre-designed Controller 
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



def control_loop():

    # set as 2 or 3 if using a Qube Servo 2 or 3 respectively
    qubeVersion = 3

    # Set as 0 if using virtual Qube Servo
    # Set as 1 if using physical Qube Servo
    hardware = 0

    # Only matters when using virtual Qube. 
    # Set as 0 for virtual DC Motor and 1 for virtual pendulum
    # KEEP AS 1, THIS EXAMPLE USES A PENDULUM
    # not important if using virtual
    pendulum = 1

    # Sample frequency of the control loop.
    frequency = 50  # 20ms

    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    # Limit sample rate for scope 
    countMax = frequency / 50
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-1, 34.75, -1.495, 3.111])
    else:
        QubeClass = QubeServo3
        # K = np.array([-1.2247, 24.9044, -0.6877, 3.1321]) # for 2ms 
        K = np.array([-1.1903, 15.4953, -0.4840, 1.1720])

    xhat = np.zeros(4, dtype=np.float64)


    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        while timeStamp < simulationTime and not KILL_THREAD:

            # sensor read             
            t1 = time.perf_counter()
            myQube.read_outputs()
            t2 = time.perf_counter()

            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition
            
            # regularization
            alpha = (alpha_f - np.pi + np.pi) % (2*np.pi) - np.pi       # alpha는 rad 단위
            alpha_degrees = abs(math.degrees(alpha))                    # 밑에 if문을 위해서 그냥 degree로 변환...

            y = np.array([theta, alpha], dtype=np.float64)

            # derivative filtering
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # negative feedback
            u = -1 * float(H @ xhat)
            # u = float(H @ xhat)
            VMAX = 10.0

            # safety check
            if alpha_degrees > 20:
                voltage = 0
                error = 0
                
            else:
                voltage = np.clip(u, -VMAX, VMAX)


            # actuator write
            myQube.write_voltage(u)
            t3 = time.perf_counter()


            time.sleep(0.01)
            print(f"Read 대기 시간: {(t2 - t1)*1000:.2f}ms | 계산+Write 시간: {(t3 - t2)*1000:.2f}ms") # time check
            

            # estimation bound
            if alpha_degrees > 5:
                xhat = np.array([0, 0, 0, 0])
            else:
                xhat = F @ xhat + G @ y


            # Plot to scopes 
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
