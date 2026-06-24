# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

**Norm-Optimal ILC (Lifted System)** for the Quanser Aero2 2-DOF helicopter. The controller improves trajectory tracking across repeated runs using a mathematically optimal feedforward correction derived from the full discrete-time lifted system model.

## Run Commands

```bash
# Run one ILC iteration (hardware required)
python ilc.py

# Automate N_ITER=50 consecutive ILC iterations with 5s reset wait between runs
python run_ilc.py

# Observer-based LQR baseline (hardcoded gains from Design.m, 50Hz)
python observer.py

# Single-shot LQR+observer baseline with data logging (dynamic design, 20Hz)
python data.py

# Plot learning progress: first vs last iteration in data/ folder
python result.py
```

```matlab
% Design.m — compute F, G, H gains → paste into observer.py
% (single-input model; different from the 2-input model in ilc.py/data.py)
```

## Architecture

### Control Structure (ilc.py / data.py)

All files share the same 4-state Aero2 model (Jp, Jy, Dp, Dy, Mg, Kpp, Kyy, Kpy, Kyp) and discretize it via ZOH at runtime using `python-control`:

```
State:  x = [theta_p, theta_y, dp, dy]'
Output: y = [theta_p, theta_y]'   (positions measured; rates estimated by observer)
Input:  u = [V_main, V_tail]'
```

Controller equation per step:
```
u(k)      = H * xhat(k) + N_bar * ref(k) + v_ilc(k)
xhat(k+1) = F_obs * xhat(k) + G_obs * y(k) + P_obs * ref(k)
```
- `F_obs = Ad - Bd@K - L@Cd`, `G_obs = L`, `P_obs = Bd @ N_bar`
- `N_bar = inv(G_dc)` where `G_dc = Cd @ inv(I - A_cl) @ Bd` (DC gain matrix for reference pre-filter)

### ILC Update Rule (Norm-Optimal, Lifted System)

The entire N-step trial is stacked into a single lifted vector and solved optimally offline before each run.

**Lifted system**: `Y_j = G_N @ U_j + d`

`G_N` is a `pN × mN` block lower-triangular Toeplitz matrix built from the augmented closed-loop Markov parameters:
```
G_N[t, s] = C_aug @ A_aug^(t-s-1) @ B_aug   for s < t,  else 0
```
where the augmented system `z = [x(4); xhat(4)]` combines plant + observer:
```
A_aug = [[Ad,         Bd @ H_ctrl],
         [G_obs @ Cd, F_obs      ]]
B_aug = [[Bd], [0]]
C_aug = [Cd, 0]
```

**Iteration-domain error dynamics**: `E_{j+1} = (I - G_N @ L_opt) @ E_j`

**Norm-optimal gain** (minimises `‖E_{j+1}‖² + λ‖ΔU_j‖²`):
```
L_opt = (G_N.T @ G_N + λI)⁻¹ @ G_N.T   [mN × pN]
```

**Per-iteration update**:
```
ΔU_vec = L_opt @ E_vec        # (mN,) ← one matrix-vector multiply
v_ilc_k = clip(v_ilc_{k-1} + ΔU.reshape(N,m), -20V, +20V)
```

`L_opt` is computed once at startup (~1–2s). `ρ(I - G_N @ L_opt)` is printed on startup; value ≈ 1.0 is expected (G_N is structurally rank-deficient at t=0 row) but ILC still converges for t ≥ 1.

**Tuning**: only `ILC_LAMBDA` (currently `1e-3`). Smaller → faster convergence, larger → more conservative.

### Reference Trajectories

**ilc.py**: Trapezoidal yaw (0→360°, 2s accel / 6s constant / 2s decel) + asymmetric-sine pitch `(sin(ωt) - 0.5*sin(2ωt))`. 20Hz, 10s trajectory.

**observer.py**: Sinusoidal multi-harmonic: yaw `A1·sin(ωt) + A2·sin(2ωt)`, pitch `B1·sin(2ωt) + B2·sin(3ωt)`. 50Hz, 10s trajectory.

**data.py**: Cosine-based `cos(a*ωt) - cos(b*ωt)` trajectory (smooth start/stop). 20Hz, 10s trajectory.

### Data Format (`data/data_k.npz`)

Keys: `U` (control inputs), `Y` (measured outputs), `R` (reference), `E` (error R-Y), `V_ILC` (ILC correction voltages), `k` (iteration index), `n_valid` (steps actually completed), `Ts` (sample time).

Partial runs (Ctrl+C or pitch safety trip) are saved with `n_valid < N_STEPS`; ILC update code zeroes out the missing portion.

### Safety

Pitch limit: `|theta_p| > 40°` → motor voltages forced to 0.0. All voltages clipped to ±20V.

### Library Path Convention

Quanser PAL/HAL imported via relative path (depth 3 from repo root):
```python
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', '00_libraries', 'python'))
```

## Key Parameters (ilc.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `FREQUENCY` | 20 Hz | Control rate (50ms period) |
| `TRAJ_TIME` | 10.0 s | Active trajectory duration |
| `RUN_TIME` | 11.0 s | Total hardware-on time |
| `ILC_LAMBDA` | 1e-3 | Regularisation — only ILC tuning knob |
| `V_ILC_MAX` | 20.0 V | Accumulated ILC voltage clamp |

## Dependencies

```
pip install control numpy matplotlib
```
Quanser PAL/HAL from `00_libraries/python` (hardware interface).
