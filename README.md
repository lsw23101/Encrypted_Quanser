# SeoulTech Encrypted Control on Quanser

## 돌리는 방법 (Qlab)

터미널 2개를 연다.

### 터미널 1 — Controller
`C:\Users\sang2\all\Encrypted_Quanser\01_Qube_Servo3\Qlab\Go\Lattigo\controller`

1. `offline.go` 파일에서 암호 파라미터와 제어기 파라미터를 설정하고 한 번 실행한다. (`go run offline.go`) — 이때 생성되는 파라미터를 다른 파일들이 공유한다.
2. `go run controller.go` 실행

### 터미널 2 — Plant
`C:\Users\sang2\all\Encrypted_Quanser\01_Qube_Servo3\Qlab\Go\Lattigo\plant`

- `python swing.py` 실행

  - `manual.py`: 이 파일을 실행한 후 Qlab 시뮬레이션에서 막대 세우는 버튼을 클릭해야 하는 파일
  - `simulation.py`, `local_20ms.py`: 디버그용
