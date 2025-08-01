# 8BitDo Controller for Isaac Lab

8BitDo Controller를 Isaac Lab에서 SE(2) 제어용 디바이스로 사용할 수 있습니다.

## 설정 방법

### 1. 디바이스 확인
```bash
# 사용 가능한 입력 디바이스 확인
evtest

# 8BitDo Controller는 보통 event16으로 나타남
# /dev/input/event16: Nintendo.Co.Ltd. Pro Controller (8BitDo 모드)
```

### 2. Isaac Lab에서 직접 사용
8BitDo Controller는 Isaac Lab의 carb 인터페이스를 통해 직접 사용됩니다.
ROS 2 Joy 노드가 필요하지 않습니다.

### 3. Isaac Lab에서 사용

#### SE(2) 제어 (2D 평면)
```python
from isaaclab.devices import Se28BitDo

# 기본 모드 (오른손 잡기)
controller = Se28BitDo(
    pos_sensitivity=0.05,    # 위치 이동 감도
    rot_sensitivity=1.6,     # 회전 감도
    dead_zone=0.01,         # 데드존
    left=0,                  # 회전 모드 (0-3)
    boost_multiplier=2.0     # 가속도 배수
)

# 90도 반시계방향 회전
controller_90 = Se28BitDo(left=1)

# 180도 반시계방향 회전  
controller_180 = Se28BitDo(left=2)

# 270도 반시계방향 회전
controller_270 = Se28BitDo(left=3)

# 메인 루프에서 사용
while True:
    base_command, gripper_state = controller.advance()
    # base_command: [x, y, yaw] 속도 명령
    # gripper_state: True/False (현재는 항상 False)
```

## 컨트롤 매핑

### 기본 모드 (`left=0`)
#### 축 제어
- **왼쪽 스틱 위**: 앞으로 이동 (X축 +)
- **왼쪽 스틱 아래**: 뒤로 이동 (X축 -)
- **왼쪽 스틱 왼쪽**: 왼쪽으로 이동 (Y축 +)
- **왼쪽 스틱 오른쪽**: 오른쪽으로 이동 (Y축 -)
- **오른쪽 스틱 왼쪽**: 반시계 방향 회전 (Yaw -)
- **오른쪽 스틱 오른쪽**: 시계 방향 회전 (Yaw +)

#### 버튼 제어
- **왼쪽 숄더 (L1)**: 가속도 모드 (기본 2배 속도)
- **A 버튼**: (미사용)
- **B 버튼**: (미사용)
- **X 버튼**: (미사용)
- **Y 버튼**: (미사용)
- **오른쪽 트리거/숄더**: (미사용)

### 90도 반시계방향 회전 (`left=1`)
컨트롤러를 90도 반시계방향으로 돌려서 잡을 때 사용합니다.

#### 축 제어 (90도 회전)
- **왼쪽 스틱 위**: 왼쪽으로 이동 (Y축 +)
- **왼쪽 스틱 아래**: 오른쪽으로 이동 (Y축 -)
- **왼쪽 스틱 왼쪽**: 뒤로 이동 (X축 -)
- **왼쪽 스틱 오른쪽**: 앞으로 이동 (X축 +)
- **오른쪽 스틱**: 회전 (동일)

### 180도 반시계방향 회전 (`left=2`)
컨트롤러를 180도 반시계방향으로 돌려서 잡을 때 사용합니다.

#### 축 제어 (180도 회전)
- **왼쪽 스틱 위**: 뒤로 이동 (X축 -)
- **왼쪽 스틱 아래**: 앞으로 이동 (X축 +)
- **왼쪽 스틱 왼쪽**: 오른쪽으로 이동 (Y축 -)
- **왼쪽 스틱 오른쪽**: 왼쪽으로 이동 (Y축 +)
- **오른쪽 스틱**: 회전 (동일)

### 270도 반시계방향 회전 (`left=3`)
컨트롤러를 270도 반시계방향으로 돌려서 잡을 때 사용합니다.

#### 축 제어 (270도 회전)
- **왼쪽 스틱 위**: 오른쪽으로 이동 (Y축 -)
- **왼쪽 스틱 아래**: 왼쪽으로 이동 (Y축 +)
- **왼쪽 스틱 왼쪽**: 앞으로 이동 (X축 +)
- **왼쪽 스틱 오른쪽**: 뒤로 이동 (X축 -)
- **오른쪽 스틱**: 회전 (동일)

#### 매핑 변환 요약
```
기본 모드 (left=0) → 90도 회전 (left=1) → 180도 회전 (left=2) → 270도 회전 (left=3)
UP → LEFT → DOWN → RIGHT
LEFT → DOWN → RIGHT → UP
DOWN → RIGHT → UP → LEFT  
RIGHT → UP → LEFT → DOWN
```

#### 가속도 기능
- **왼쪽 숄더 (L1) 버튼**: 가속도 모드 활성화
- **가속도 배수**: 기본 2배 (boost_multiplier로 조정 가능)
- **적용 범위**: 모든 이동 및 회전에 적용
- **동작 방식**: 버튼을 누르고 있는 동안만 가속도 적용

#### 동작 방식
- **1.0 입력**: 해당 방향으로 이동 시작
- **0.0 입력**: 즉시 정지
- **동시 입력**: 여러 방향 동시 이동 가능
- **누적 방식**: 각 방향별로 독립적인 상태 관리
- **가속도**: 왼쪽 숄더 버튼으로 속도 증가

## 특징

- **높은 정밀도**: 16비트 축 해상도
- **직관적인 조작**: 게임패드 스타일의 익숙한 조작감
- **SE(2) 2D 평면 제어**: 위치(X,Y) + 회전(Yaw)
- **즉시 반응**: 누르면 이동, 떼면 정지
- **동시 입력**: 여러 방향 동시 제어 가능
- **carb 기반**: Isaac Lab과 완전 통합

## 사용 예시

### Isaac Lab에서 로봇 제어
```python
from isaaclab.devices import Se28BitDo

# 컨트롤러 생성
controller = Se28BitDo(
    pos_sensitivity=0.05,
    rot_sensitivity=1.6,
    dead_zone=0.01
)

# 메인 루프
while simulation_app.is_running():
    # 컨트롤러 명령 가져오기
    base_command, gripper_state = controller.advance()
    
    # 로봇에 명령 전달
    # base_command[0]: 앞/뒤 이동
    # base_command[1]: 좌/우 이동  
    # base_command[2]: 회전
    
    # 시뮬레이션 스텝
    obs, _, _, _, _ = env.step(action)
```

## 문제 해결

### 권한 문제
```bash
# udev 규칙 추가 (선택사항)
sudo nano /etc/udev/rules.d/99-pro-controller.rules

# 다음 내용 추가:
KERNEL=="event*", ATTRS{name}=="Nintendo.Co.Ltd. Pro Controller", MODE="0666"
```

### 디바이스 인식 안됨
```bash
# 입력 디바이스 목록 확인
ls -la /dev/input/event*

# 권한 확인
ls -la /dev/input/event16
```

### 컨트롤러가 반응하지 않음
1. **데드존 확인**: `dead_zone` 값을 늘려보세요 (0.01 → 0.05)
2. **감도 조정**: `pos_sensitivity`, `rot_sensitivity` 값을 조정해보세요
3. **입력 이벤트 확인**: 디버그 출력으로 입력이 들어오는지 확인

## 완성! carb 기반 8BitDo Controller 디바이스

성공적으로 **carb 기반** 8BitDo Controller를 구현했습니다! 🎮

### 🔄 **주요 특징**

**1. carb 기반 구조**:
- `carb.input` 직접 사용
- Isaac Lab DeviceBase 상속
- Omniverse와 완전 통합

**2. 직관적인 제어**:
- 누르면 이동, 떼면 정지
- 각 방향별 독립적인 상태 관리
- 동시 입력 지원

**3. 유연한 설정**:
- `pos_sensitivity`: 위치 이동 감도
- `rot_sensitivity`: 회전 감도  
- `dead_zone`: 데드존 설정

### 📁 **파일 구조**

```
source/isaaclab/isaaclab/devices/micro_8bitdo/
├── __init__.py              # Se28BitDo 클래스 export
├── se2_8bitdo.py           # carb 기반 SE2 제어
└── README.md               # 사용법 가이드
```

### 🎮 **사용 방법**

**Isaac Lab에서 사용**:
```python
from isaaclab.devices import Se28BitDo

controller = Se28BitDo()
while True:
    base_command, gripper_state = controller.advance()
    # base_command: [x, y, yaw] 속도 명령
```

이제 8BitDo Controller가 Isaac Lab에서 완벽하게 작동합니다! 🎮🤖 