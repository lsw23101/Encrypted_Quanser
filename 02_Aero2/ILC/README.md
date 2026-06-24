# Encrypted ILC — Quanser Aero2

Homomorphic-encryption-based Norm-Optimal Iterative Learning Control (ILC) for the Quanser Aero2 2-DOF helicopter.  
All ILC state updates run entirely in the ciphertext domain using RLWE encryption via [Lattigo v6](https://github.com/tuneinsight/lattigo).

---

## Repository Structure

```
.
├── Virtual_simulation/      # Hardware experiment (Aero2 + TCP)
│   ├── TCP.py               #   Python hardware client
│   ├── EncILC.go            #   Go encrypted ILC server (full-space)
│   ├── EncILC_svd.go        #   Go encrypted ILC server (SVD reduced-space)
│   └── result.py            #   Plot learning progress from saved data
│
├── Paper_plot/              # Offline virtual simulations for paper figures
│   ├── simulation_svd.go
│   ├── simulation_kernel.go
│   ├── simulation_packed.go
│   ├── simulation_nopacking.go
│   ├── test.go
│   ├── conversion.m
│   ├── controller_data/
│   ├── plot_svd.py
│   ├── plot_kernel.py
│   ├── plot_test.py
│   └── compare_ilc_timing.py
│
├── quanser_lib/             # Quanser PAL/HAL Python bindings (see License below)
│   ├── pal/
│   ├── hal/
│   ├── pit/
│   └── qvl/
│
└── img/                     # Figures for paper/slides
```

---

## Quick Start — Hardware Experiment

### Prerequisites

**Python**
```bash
pip install numpy matplotlib
```

**Go dependencies** (in `Virtual_simulation/`)
```bash
go get github.com/tuneinsight/lattigo/v6
go get github.com/CDSL-EncryptedControl/CDSL
```

### Running

Start the Go server first (takes ~30s for offline setup), then the Python client:

```bash
# Terminal 1 — encrypted ILC server
cd Virtual_simulation
go run EncILC_svd.go

# Terminal 2 — hardware client
cd Virtual_simulation
python TCP.py
```

On each completed trial, results are saved to `Virtual_simulation/data/data_k.npz`.

### Plot learning progress

```bash
cd Virtual_simulation
python result.py
```

---

## Quick Start — Virtual Simulation (Paper Plots)

```bash
cd Paper_plot

# Run MATLAB once to generate controller matrices
# matlab -r "run('conversion.m'); exit"

# SVD reduced-space encrypted ILC (5 000 trials)
go run simulation_svd.go

# Full-space encrypted ILC (5 000 trials)
go run simulation_kernel.go

# Timing benchmarks
go run simulation_packed.go
go run simulation_nopacking.go

# Plot results
python plot_svd.py
python plot_kernel.py
python compare_ilc_timing.py
```

---

## ILC Algorithm

Norm-optimal update (lifted system):

```
L_opt = (G_N^T G_N + λI)^{-1} G_N^T
ΔU    = L_opt · E
```

All multiplications execute in the ciphertext domain. `λ = 1e-3` is the only tuning parameter.

---

## License

### This repository (MIT)

Copyright (c) 2026 Sangwon Lee (CDSL, Seoul National University of Science and Technology)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

### quanser_lib/ — Quanser Research License

The `quanser_lib/` directory contains Python bindings (PAL/HAL) from the **Quanser Academic Research Suite**, distributed by [Quanser Inc.](https://www.quanser.com) under a **research-only license**.

- Redistribution is permitted solely within **private repositories** for academic and research purposes.
- Commercial use, public redistribution, or sublicensing of `quanser_lib/` is **not permitted**.
- All rights to `quanser_lib/` remain with Quanser Inc.

If you do not have a valid Quanser license, remove `quanser_lib/` and install the Quanser Academic Suite separately from your institution.
