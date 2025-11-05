# Qube-Servo 3 — Encrypted Control

전체 틀:
<플랜트 = 실험장치 = python code> : 
장치로 엔코더 각도 2개 측정 - tcp 통신을 통해 y 송신 - tcp 통신을 통해 u 수신 - 모터에 전압 인가

<컨트롤러 = 암호 제어 코드 = 이 작업공간에서는 go 언어> : 
tcp로 전달받은 float 데이터 수신 - { 암호화 - 암호 제어 연산 - 복호화   (각 연구자들의 방식으로)} - tcp float 데이터 송신

각 팀은 {} 의 작업을 수행 
{} 과정에 걸리는 시간을 플랜트의 샘플링 주기로 미리 설정


# CDSL 라이브러리 기반 Go code

해야 할 일:

0. 샘플링 주기를 설정

1. conversion.m 파일을 통해 제어기 설계 및 변형 (ARX형태의 Hu Hy 혹은 재암호화를 위한 F G H R H 행렬) 

2. 위 결과를 server.go 코드에서 복붙 & 암호 파라미터 적절히 수정

3. 실험


### 설명
- 센서 `y`, 제어입력 `u`, 컨트롤러 상태 `x`를 **RLWE** 암호문으로 처리합니다.
- 컨트롤러는 오프라인에서 스케일/패킹을 준비한 뒤, 루프에서 다음 순서를 반복합니다:  
  1) TCP로 `y=[theta, alpha]` 수신  
  2) `y` 양자화·암호화  
  3) 암호 제어 연산
  4) `u` 복호화  
  5) `u`(스칼라) 한 줄 송신  
  6) `u` 재암호화  
  7) `x ← Fx + Gy + Ru` 상태 업데이트 or `x ← Hu * u_seq + Hy * y_seq` 
- 통신 포맷: 클라이언트가 `"theta,alpha\n"` 전송 → 서버는 `"u\n"`(스칼라) 응답  
- 기본 포트: `127.0.0.1:9000`



### Usage

**터미널 1**
```bash
$ go run ARX_server.go
or
$ go run RGSW_server.go
```

**터미널 2**

```bash
$ python sensor_actuator.py
```


