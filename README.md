# SeoulTech Encrypted Control on Quanser

This repository contains the implementation of encrypted control for the Quanser Qube Servo 3 (Rotary Inverted Pendulum).  

See demonstration video !
https://www.youtube.com/shorts/artZGFfgOVo


### 00_CDSL: Cryptography for Dynamic Systems Library
* **Reference:** RLWE-based encrypted control algorithms and baseline codes designed by the Control and Dynamics Systems Lab (CDSL) at Seoul National University.  

https://github.com/CDSL-EncryptedControl/CDSL


### 01_Encrypted Control
Applies the CDSL encrypted control library to actual Quanser hardware.
Utilizes a re-encryption method combined with time scheduling to ensure only a single send/receive transmission occurs per control step.

Plant and controller communicate via **TCP** (single PC, split processes).

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
  * **Plaintext** — Pure Python plaintext baseline with same TCP structure
    * `controller.py`: Python equivalent of Go/Plaintext controller, communicates over TCP.
    * `swing.py` / `manual.py` / `simulation.py`: Plant-side scripts.
    * `local_20ms.py`: Combined plant + controller in a single process (no TCP, for local testing).
        
### 02_Modeling
Contains foundational knowledge, dynamic equations, and hardware parameters required to operate the Quanser Qube Servo 3.
* Includes parameter comparisons and dynamic models for both Qube Servo 2 (QS2) and Qube Servo 3 (QS3).
* Stores the baseline controller designs and the `local.py` script for local hardware testing.
