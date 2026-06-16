# SeoulTech Encrypted Control on Quanser

## 돌리는 방법 (Qlab)

저장소를 클론한 위치를 기준으로, 터미널 2개를 열어서 각각 controller 폴더와 plant 폴더에서 실행한다.

### 터미널 1 — `01_Qube_Servo3/Qlab/Go/Lattigo/controller` 폴더에서 실행

1. `offline.go` 파일에서 암호 파라미터와 제어기 파라미터를 설정하고 한 번 실행한다. (`go run offline.go`) — 이때 생성되는 파라미터를 다른 파일들이 공유한다. (리포 기준 이미 실행 된 상태, 파라미터 변경이 필요할 시 변경 후 실행)
2. `go run controller.go` 실행

### 터미널 2 — `01_Qube_Servo3/Qlab/Go/Lattigo/plant` 폴더에서 실행

- `python swing.py` 실행

  - `manual.py`: 이 파일을 실행한 후 Qlab 시뮬레이션에서 막대 세우는 버튼을 클릭해야 하는 파일
  - `simulation.py`, `local_20ms.py`: 디버그용
