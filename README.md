# Quanser 관련 작업 정리 




''
### 0. CDSL
서울대학교 CDSL 연구팀 라이브러리

### 1. Quanser
CDSL 라이브러리 기반으로 Quanser 실험장비에 암호 제어 적용
재암호화 방법 적용 + time scheduling 으로 통신은 송수신 한번씩 수행

- controller/controller.go : x_c unpack + u = Hx 계산 후 송신 - y 수신 - state update  
- plant/swing.py : swing up - Full state LQR (1s) - Enc. control
- plant/real.py : 위에서 스윙업 없음 (손으로 올리기)
- plant/simulation.py : 디버그용 시뮬레이션

샘플링시간 20ms  
10번해서 10번 1분 이상 잘 돌아감 (스윙업이 이상할 경우에 선정리)

추후 Lattigo 폴더 이외의 다른 연구팀의 work을 추가할 계획

### 2. Datadrvien
데이터 기반 제어를 하기 위해서 데이터를 얻는 코드와 그것으로 컨트롤러 설계 (poleplacement) 및 시뮬레이션과 그 제어기로 실험 했을 시 제어가 가능함  

### 3. Enc_control (테스트 용...)
RGSW와 RLWE를 각각 10ms 5ms에서 돌려봄 이때 통신은 평문으로 통신하고 암호화 - 연산 - 복호화를 모두 컨트롤러 코드에서 수행함  
한번씩 값이 튀게 되면서 제어가 안되는 상황이 발생  
TODO:
- 위 예외 상황의 원인을 찾고 해결하기  
- RCF 방식의 제어기도 똑같이 돌려보기  

### 4. Swingup
매트랩 상에서 동역학 수식을 이용한 비선형 회전 도립진자의 시뮬레이션 환경을 구현 (동일한 파라미터로)  
위 시뮬레이션을 바탕으로 실험 장비 위에 올리는 코드 swing.py 로 수행  
20ms 샘플링 시간과 150의 mu 파라미터 그리고 6v의 전압 한계 설정으로 스윙업이 되는 것을 확인하였음  
샘플링 타임을 늘렸을때 위 파라미터의 수정이 필요하며 50ms 에서는 아직 성공하지 못함  
TODO:
- 느린 샘플링 시간에서 스윙업이 되도록...  
- 지금의 간단한 스위칭 로직 더 좋은 알고리즘 적용...

### 5. Model
Quanser Qube Servo 3 실험 장비를 구동하기 위해 필요한 선행 지식 및 파라미터 등의 자료  
qs2와 qs3에서의 각각 파라미터와 동역학 파라미터  
그리고 실험 장비에 올리는 컨트롤러 설계와 local.py 코드 저장용  
''
