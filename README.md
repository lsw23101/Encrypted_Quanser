# Encrypted Control on Quanser Qube Servo 3

Ring-LWE(RLWE/RGSW) 동형암호를 이용해 Quanser Qube Servo 3(도립진자)를 **암호화된 상태에서 제어**하는 구현입니다.  
CDSL(Control and Dynamics Systems Lab, SNU)의 암호제어 라이브러리를 기반으로 합니다.

---

## 구조

```
01_Qube_Servo3/Qlab/Go/
  PlainComm/          ← 통신은 평문, 제어 연산은 암호문 (주 실행 버전)
    encrypt.go        ← 암호화 제어기 (Go에서 암호화·연산·복호화 수행)
    non_encrypt.go    ← 평문 제어기 (비교용 baseline)
    swing.py          ← 하드웨어 인터페이스 (swing-up → 암호제어)
    manual.py         ← 수동 세우기 후 제어 시작
  EncComm/            ← 통신도 암호문 (TCP로 암호문을 직접 송수신)
    controller/       ← Go 암호화 제어기
    plant/            ← Python 하드웨어 인터페이스
    crypto/           ← Python FFI용 C-shared 라이브러리
```

### PlainComm vs EncComm

| | PlainComm | EncComm |
|---|---|---|
| TCP 통신 | **평문** float64 | **암호문** (RLWE ciphertext bytes) |
| 암호화 위치 | Go 제어기 내부 | Python(plant)에서 암호화 후 전송 |
| 사용 목적 | 주 데모 / 성능 비교 | 완전한 암호 통신 파이프라인 |

EncComm은 Python이 직접 Lattigo 공유 라이브러리(`.dll`/`.so`)를 로드해 암호화하고 암호문을 TCP로 주고받는 구조입니다.

---

## 실행 방법 (PlainComm 기준)

> 사전 준비: Quanser Interactive Labs 실행, Go 설치, Python 의존성 설치 완료 가정

### 암호화 제어기 (`encrypt.go` + `swing.py`)

**터미널 1** — Go 암호화 제어기 시작

```bash
cd 01_Qube_Servo3/Qlab/Go
go run ./PlainComm/encrypt.go
```

키 생성 및 행렬 암호화가 완료되면 `--- Ready ---` 출력 후 연결 대기.

**터미널 2** — Python 하드웨어 인터페이스

```bash
cd 01_Qube_Servo3/Qlab/Go/PlainComm
python swing.py
```

swing-up → LQR 안정화 → 암호화 제어 순으로 자동 전환.  
수동으로 세운 후 바로 암호화 제어를 시작하려면 `manual.py`를 사용.

### 평문 제어기 (`non_encrypt.go` + `swing.py`)

**터미널 1**

```bash
cd 01_Qube_Servo3/Qlab/Go
go run ./PlainComm/non_encrypt.go
```

**터미널 2**

```bash
cd 01_Qube_Servo3/Qlab/Go/PlainComm
python swing.py
```

`swing.py`는 두 제어기 모두와 호환됩니다.

---

## 통신 프로토콜 (PlainComm)

매 제어 주기(20 ms)마다:

1. **Controller → Plant**: 평문 `u` (float64 × 1)
2. **Plant → Controller**: 평문 `y = [θ, α]` (float64 × 2)

`encrypt.go`는 내부에서 `y`를 RLWE 암호화하고, 암호문 연산으로 `u = H·x`를 계산한 뒤 복호화해서 전송합니다.

---

## 제어기 파라미터 (not secure)

| 항목 | 값 |
|---|---|
| 제어 주기 | 20 ms (50 Hz) |
| RLWE 다항식 차수 | N = 2¹⁰ = 1024 |
| 상태 차원 n | 4 |
| 출력 차원 p | 2 (θ, α) |
| 입력 차원 m | 1 (전압 u) |

---

## 02_Aero2 — Encrypted ILC (Quanser Aero2)

Quanser Aero2 2DOF 헬리콥터를 대상으로 한 **암호화 ILC(Iterative Learning Control)** 구현입니다.  
반복 시행마다 궤적 추종 오차를 줄이는 Norm-Optimal ILC를 동형암호 위에서 수행합니다.

### 실행 방법 (Qlab 기준)

**터미널 1** — Go 암호화 ILC 서버 (전체 실험 동안 유지)

```bash
cd 02_Aero2/ILC/Virtual_simulation
go run EncILC_svd.go
```

오프라인 셋업 완료 후 `--- Ready ---` 출력까지 약 30초 소요.

**터미널 2** — Python 하드웨어 클라이언트 (trial마다 반복 실행)

```bash
cd 02_Aero2/ILC/Virtual_simulation
python TCP.py
```

**반복 절차 (trial k → k+1):**
1. `TCP.py` 실행 → Qlab에서 trial 진행
2. trial 종료 후 Qlab에서 **Reset 버튼** 눌러 초기 상태로 복귀
3. `python TCP.py` 다시 실행 → 반복

서버(터미널 1)는 계속 켜둔다. trial 결과는 `data/data_k.npz`에 자동 저장.

---

## 참고

- CDSL 암호제어 라이브러리: https://github.com/CDSL-EncryptedControl/CDSL
- Lattigo (RLWE/RGSW 라이브러리): https://github.com/tuneinsight/lattigo
