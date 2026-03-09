# SeoulTech Encrypted Control on Quanser

This repository contains the implementation of encrypted control algorithms for the Quanser Qube Servo 3 (Rotary Inverted Pendulum).
(00, 01 for Encyrpted Control and 02-05 for studying)

See demonstration video !
https://www.youtube.com/shorts/artZGFfgOVo


### 0. CDSL Library
* **Reference:** RLWE-based encrypted control algorithms and baseline codes designed by the Control and Dynamics Systems Lab (CDSL) at Seoul National University.

### 1. Quanser Implementation
Applies the CDSL encrypted control library to actual Quanser hardware. 
* **Communication Optimization:** Utilizes a re-encryption method combined with time scheduling to ensure only a single send/receive transmission occurs per control step.
* **Core Modules:**
  * `controller/controller.go`: Unpacks the controller state $x_c$, computes the control input $u = Hx$, transmits the data, receives the sensor output $y$, and updates the state.
  * `plant/swing.py`: Executes the full hardware sequence: Swing-up -> Full-state LQR (1 sec) -> Encrypted control.
  * `plant/real.py`: Runs encrypted control without the automated swing-up (requires manual stabilization to the upright position).
  * `plant/simulation.py`: Simulation environment for debugging purposes.
* **Performance:** Operates reliably at a **20ms sampling time**. Achieved a 100% success rate (10/10 runs) maintaining control for over 1 minute (provided hardware wiring is properly managed to avoid swing-up interference).
* **Future Work:** Plan to integrate implementations from other research groups beyond the current Lattigo-based framework.

### 2. Data-Driven Control
Modules dedicated to data-driven control methodologies.
* Contains scripts for system data acquisition.
* Utilizes the collected data to design controllers (e.g., pole placement) and run simulations.
* Successfully validated the data-driven controllers on the physical hardware.

### 3. Enc_Control (Experimental Testing)
An experimental testbed for evaluating RGSW and RLWE schemes at faster sampling times (10ms and 5ms).
* **Current Setup:** Communication is handled in plaintext. The entire process (Encryption -> Computation -> Decryption) is executed locally within the controller code to measure pure computational overhead.
* **Known Issues:** Encountered occasional value spikes (glitches) that lead to system instability and loss of control.
* **TODO:**
  - [ ] Identify and resolve the root cause of the unexpected value spikes.
  - [ ] Implement and test an RCF (Rational Controller Form) based controller under the same high-speed conditions.

### 4. Swing-up Control
Focuses on the nonlinear dynamics and swing-up logic for the rotary inverted pendulum.
* Designed a simulation environment in MATLAB using dynamic equations with matching hardware parameters.
* `swing.py` deploys this validated logic onto the actual hardware.
* Successfully achieved automated swing-up at a **20ms sampling time**, using parameter `mu = 150` and a **6V voltage limit**.
* **Known Issues:** Swing-up logic fails at slower sampling rates (e.g., 50ms) and requires parameter retuning.
* **TODO:**
  - [ ] Achieve successful swing-up at slower sampling times.
  - [ ] Replace the current basic switching logic with a more advanced and robust algorithm.

### 5. System Models & Parameters
Contains foundational knowledge, dynamic equations, and hardware parameters required to operate the Quanser Qube Servo 3.
* Includes parameter comparisons and dynamic models for both Qube Servo 2 (QS2) and Qube Servo 3 (QS3).
* Stores the baseline controller designs and the `local.py` script for local hardware testing.
