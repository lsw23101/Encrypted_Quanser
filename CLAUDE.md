# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository implements **encrypted dynamic control** for the Quanser Qube Servo 3 (Rotary Inverted Pendulum) using Ring-LWE (RLWE/RGSW) homomorphic encryption. The controller operates on encrypted sensor data without decryption. The CDSL (Control and Dynamics Systems Lab, Seoul National University) encrypted control algorithms are the cryptographic foundation.

Reference library: https://github.com/CDSL-EncryptedControl/CDSL

## Repository Structure

```
00_libraries/           # Quanser PAL/HAL/QVL Python libraries and Simulink models
01_Qube_Servo3/
  Go/
    Lattigo/            # Encrypted controller (RLWE/RGSW via Lattigo)
      controller/       # Go encrypted controller
      plant/            # Python hardware interface scripts
      crypto/           # C-shared library wrapping Lattigo for Python FFI
      utils/
    Plaintext/          # Plaintext baseline with same TCP structure
    utils/
    go.mod / go.sum
  Python/
    Plaintext/          # Pure Python plaintext baseline
02_Aero2/
  Python/
    Plaintext/          # Aero2 platform demos
03_Modeling/            # QS2/QS3 state-space models, hardware parameters
```

## Build & Run Commands

### Go (01_Qube_Servo3/Go/)
```bash
# Run encrypted controller (one terminal)
cd 01_Qube_Servo3/Go/Lattigo/controller
go run controller.go offline.go

# Run plaintext Go controller (one terminal)
cd 01_Qube_Servo3/Go/Plaintext
go run controller.go

# Build C-shared library for Python FFI (Linux/macOS → .so, Windows → .dll)
cd 01_Qube_Servo3/Go/Lattigo/crypto
go build -o client_crypto.so -buildmode=c-shared lib_crypto.go
```

### Python plant interface (separate terminal from Go controller)
```bash
cd 01_Qube_Servo3/Go/Lattigo/plant
python swing.py      # Full sequence: swing-up → LQR → encrypted control
python manual.py     # Encrypted control only (manual swing-up required)
python simulation.py # Simulation for debugging
```

### Standalone single-process test (no TCP, no hardware required)
```bash
cd 01_Qube_Servo3/Python/Plaintext
python local_20ms.py
```

## Architecture

### Encrypted Control Flow

The system uses **re-encryption scheduling** so only one send/receive TCP transmission occurs per control step:

1. **Plant side** (`plant/swing.py`): reads hardware sensors → encrypts measurement `y` → sends to controller → receives encrypted `u` → decrypts → applies to hardware
2. **Controller side** (`controller/controller.go`): receives `y_enc` → computes `u = H·x_c` over encrypted data (RLWE/RGSW) → sends `u_enc` → updates state `x_c' = F·x_c + G·y`
3. **Offline setup** (`controller/offline.go`): loads pre-computed encrypted RGSW matrices and evaluation keys from `enc_data/`

### Controller Design (conversion.m)
MATLAB scripts compute the re-encryption based controller matrices (F, G, H, R) from the plant model. Run before deploying; outputs `.mat` files loaded by Go at startup.

### Python ↔ Go Bridge
`01_Qube_Servo3/Go/Lattigo/crypto/lib_crypto.go` is compiled as a C-shared library (`client_crypto.so`/`.dll`) to expose Lattigo encryption/decryption to Python via ctypes FFI.

### Two Parallel Implementations
- **Go/Lattigo**: Full homomorphic encryption — the actual contribution
- **Go/Plaintext** and **Python/Plaintext**: Identical TCP structure but plaintext arithmetic — used for performance comparison and debugging

## Library Import Convention

All Python files import Quanser PAL/HAL from `00_libraries/python` using a path relative to `__file__`:

```python
# Files in plant/ (depth 4 from repo root)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '00_libraries', 'python'))

# Files at depth 3 (Go/Plaintext/, Python/Plaintext/, 02_Aero2/Python/Plaintext/)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '00_libraries', 'python'))
```

## Dependencies

Go projects use `github.com/tuneinsight/lattigo/v6 v6.1.0` (BGV/RLWE/RGSW homomorphic encryption). `go.mod` is at `01_Qube_Servo3/Go/go.mod`.

Hardware interface uses Quanser PAL/HAL Python libraries in `00_libraries/python/`.

## Key Parameters

- Control period: 20 ms (50 Hz)
- RLWE polynomial degree: N = 2^12
- TCP communication: single PC, two processes, little-endian double precision frames
- Hardware: Quanser Qube Servo 3 — arm length 85 mm, pendulum 129 mm
