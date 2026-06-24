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

**Terminal 1** — Go 암호화 ILC 서버 (한 번만 실행, 전체 실험 동안 유지)

```bash
cd Virtual_simulation
go run EncILC_svd.go
```

오프라인 셋업 완료 후 `--- Ready ---` 출력까지 약 30초 소요.  
서버는 trial이 끝나도 ILC 상태(암호화된 feedforward)를 누적하며 계속 대기한다.

**Terminal 2** — Python 하드웨어 클라이언트 (trial마다 반복 실행)

```bash
cd Virtual_simulation
python TCP.py
```

**반복 절차 (trial k → k+1):**

1. `TCP.py` 실행 → Qlab에서 trial 진행 (10초)
2. trial 종료 후 Qlab에서 **Reset 버튼** 눌러 초기 상태로 복귀
3. Terminal 2에서 `python TCP.py` 다시 실행
4. 반복

각 trial 결과는 `Virtual_simulation/data/data_k.npz`에 자동 저장된다.  
Terminal 1의 서버는 계속 켜둔다.

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
