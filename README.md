# SeoulTech Encrypted Control on Quanser

This repository contains the implementation of encrypted control for the Quanser Qube Servo 3 (Rotary Inverted Pendulum) and plaintext baseline controllers for the Quanser Aero 2 (2-DOF Helicopter).

See demonstration video !
https://www.youtube.com/shorts/artZGFfgOVo

> **Reference — CDSL (Cryptography for Dynamic Systems Library)**
> RLWE-based encrypted control algorithms by the Control and Dynamics Systems Lab (CDSL), Seoul National University.
> https://github.com/CDSL-EncryptedControl/CDSL


### 00_libraries
Quanser PAL/HAL/QVL Python libraries and Simulink models for Quanser hardware.


### 01_Qube_Servo3
Applies the CDSL encrypted control library to the Quanser Qube Servo 3 (Rotary Inverted Pendulum).
Utilizes a re-encryption method combined with time scheduling to ensure only a single send/receive transmission occurs per control step.

Plant and controller communicate via **TCP** (single PC, split processes).

Each environment (Qlab virtual / Hardware) shares the same code structure:

* **Qlab** — Quanser QLab virtual environment
* **Hardware** — Physical Qube Servo 3 hardware

Both contain:

* **Go:**
  * **Lattigo** — Encrypted controller using RLWE/RGSW
    * controller
      * `controller.go`: Unpacks the controller state $x_c$, computes the control input $u = Hx$, transmits the data, receives the sensor output $y$, and updates the state.
      * `offline.go`: Restore encrypted RGSW matrices, RLWE initial state and some evaluation keys.
      * `conversion.m`: Design re-encryption based controller (F, G, H, R).
    * plant
      * `swing.py`: Executes the full hardware sequence: Swing-up → Full-state LQR (1 sec) → Encrypted control.
      * `manual.py`: Runs encrypted control without the automated swing-up (requires manual stabilization to the upright position).
      * `simulation.py`: Simulation environment for debugging purposes.
    * crypto: Build files to use Lattigo in Python

  * **Plaintext** — Plaintext baseline (no encryption) with same TCP structure
    * `controller.go`: Computes $u = Hx_c$, updates state via $x_c' = Fx_c + Gy$, communicates over TCP.
    * `swing.py` / `manual.py` / `simulation.py`: Same plant-side scripts as Lattigo version.

* **Python:**
  * **Plaintext** — Pure Python plaintext baseline
    * `controller.py`: Python equivalent of Go/Plaintext controller, communicates over TCP.
    * `swing.py` / `manual.py` / `simulation.py`: Plant-side scripts.
    * `local_20ms.py`: Combined plant + controller in a single process (no TCP, for local testing).


### 02_Aero2
Quanser Aero 2 (2-DOF helicopter) controllers.

* **Qlab** — Quanser QLab virtual environment
  * **Python/Plaintext**
    * `model.m`: 4th-order MIMO state-space model and LQR controller design (Fellag et al., 2024). Linearized about hover equilibrium; includes cross-coupling between pitch and yaw channels.
    * `fullstate.py`: Full-state feedback LQR tracking control. Tracks a step reference (pitch ±30°, yaw ∓30°) with feedforward precompensator $N_{bar}$; reference is flipped at t=10 s to verify symmetric tracking.

* **Hardware** — Physical Aero 2 hardware
  * **Python/Plaintext**
    * `model.m`: Same controller design script as Qlab.
    * `fullstate.py`: Full-state feedback LQR tracking control on physical hardware. Direct encoder/rate measurements (no observer); pitch safety cutoff at ±40°. Tracks step reference (pitch 30°, yaw −30°) flipping at t=10 s.

  ![Reference tracking result — hardware experiment](02_Aero2/Hardware/0325_ref_trac_hardware.png)

* **quanser_resource** — Quanser official sample scripts
  * `aero2_1dof_rotor_0_pi_control_immediate.py`: PI controller for rotor 0 (1-DOF, immediate I/O mode, 150 Hz).
  * `aero2_read_all_sensor_data_task.py`: Read all sensor data in task-based mode.
  * `aero2_read_all_sensor_data_task_qscope.py`: Sensor data with real-time QScope visualization.


### 03_Modeling
Dynamic models and hardware parameters for Quanser platforms.

* **Aero2**
  * Reference paper: *2-DOF Helicopter Control Via State Feedback and Full/Reduced-Order Observers*
  * Quanser official lab materials (l0–l9): hardware interfacing, block diagram modeling, rotor step response, pitch parameter estimation, PID design, gain scheduling — each with `digital_twin` and `hardware` versions.

* **Qube_servo3**
  * State-space models and parameter files for Qube Servo 2 (QS2) and Qube Servo 3 (QS3).
  * Controller design scripts and Quanser SP5 pendulum modeling lab materials.
