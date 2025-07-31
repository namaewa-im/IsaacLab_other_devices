# 8BitDo Controller for Isaac Lab

8BitDo Controller를 Isaac Lab에서 SE(2) 및 SE(3) 제어용 디바이스로 사용할 수 있습니다.

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

#### SE(3) 제어 (3D 공간)
```python
from isaaclab.devices import Se38BitDo

# 8BitDo Controller 디바이스 생성 (SE3)
controller = Se38BitDo()

# 메인 루프에서 사용
while True:
    delta_pose, gripper_state = controller.advance()
    # delta_pose: [x, y, z, roll, pitch, yaw] 변화량
    # gripper_state: True/False (그리퍼 열림/닫힘)
```

#### SE(2) 제어 (2D 평면)
```python
from isaaclab.devices import Se28BitDo

# 8BitDo Controller 디바이스 생성 (SE2)
controller = Se28BitDo()

# 메인 루프에서 사용
while True:
    delta_pose, gripper_state = controller.advance()
    # delta_pose: [x, y, 0, 0, 0, yaw] 변화량 (2D 평면)
    # gripper_state: True/False (그리퍼 열림/닫힘)
```

## 컨트롤 매핑

### SE(3) 제어 (3D 공간)
#### 축 제어
- **왼쪽 스틱**: X, Y축 이동
- **오른쪽 스틱**: Pitch, Yaw 회전
- **D-pad**: Roll 회전 (좌우), Z축 이동 (상하)

#### 버튼 제어
- **A 버튼**: 그리퍼 토글 (열림/닫힘)
- **B 버튼**: Z축 상승
- **X 버튼**: Z축 하강
- **Y 버튼**: (미사용)

### SE(2) 제어 (2D 평면)
#### 축 제어
- **왼쪽 스틱**: X, Y축 이동
- **오른쪽 스틱**: Yaw 회전 (Z축 회전)
- **D-pad**: 미세 조정 (좌우/상하)

#### 버튼 제어
- **A 버튼**: 그리퍼 토글 (열림/닫힘)
- **B 버튼**: 빠른 시계방향 회전
- **X 버튼**: 빠른 반시계방향 회전
- **Y 버튼**: (미사용)

## 특징

- **높은 정밀도**: 16비트 축 해상도 (-32767 ~ 32767)
- **직관적인 조작**: 게임패드 스타일의 익숙한 조작감
- **SE(3) 완전한 6DOF 제어**: 위치(X,Y,Z) + 회전(Roll,Pitch,Yaw)
- **SE(2) 2D 평면 제어**: 위치(X,Y) + 회전(Yaw)
- **그리퍼 제어**: 버튼으로 그리퍼 열림/닫힘 토글
- **유연한 제어**: SE(2)와 SE(3) 모두 지원

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

## 완성! carb 기반 8BitDo Controller 디바이스

성공적으로 ROS 2 기반에서 **carb 기반**으로 변경했습니다! 🎮

### 🔄 **주요 변경사항**

**1. ROS 2 → carb 변경**:
- `rclpy` → `carb.input`
- `sensor_msgs.msg.Joy` → `carb.input.GamepadInput`
- ROS 2 Node → Isaac Lab DeviceBase

**2. carb 기반 구조**:
```python
import carb
import omni

# Omniverse 인터페이스 획득
self._appwindow = omni.appwindow.get_default_app_window()
self._input = carb.input.acquire_input_interface()
self._gamepad = self._appwindow.get_gamepad(0)

# 이벤트 구독
self._gamepad_sub = self._input.subscribe_to_gamepad_events(
    self._gamepad,
    lambda event, *args, obj=weakref.proxy(self): obj._on_gamepad_event(event, *args),
)
```

### 📁 **업데이트된 파일 구조**

```
source/isaaclab/isaaclab/devices/8bitdo/
├── __init__.py              # SE2, SE3 클래스 export
├── se2_8bitdo.py           # carb 기반 SE2 제어
├── se3_8bitdo.py           # carb 기반 SE3 제어
├── test.py                  # carb 기반 테스트
├── joy_config.yaml          # (더 이상 불필요)
└── README.md               # carb 기반 사용법
```

### 📝 **사용 방법**

**SE2 테스트**:
```bash
python source/isaaclab/isaaclab/devices/8bitdo/test.py se2
```

**SE3 테스트**:
```bash
python source/isaaclab/isaaclab/devices/8bitdo/test.py se3
```

**Isaac Lab에서 사용**:
```python
from isaaclab.devices import Se28BitDo, Se38BitDo

# SE2 제어 (2D 평면)
controller_se2 = Se28BitDo()

# SE3 제어 (3D 공간)
controller_se3 = Se38BitDo()

# 메인 루프
while True:
    delta_pose, gripper_state = controller.advance()
```

### 🎮 **carb 기반의 장점**

1. **ROS 2 의존성 제거**: 별도 Joy 노드 불필요
2. **더 빠른 응답**: 직접 하드웨어 접근
3. **Isaac Lab 완전 통합**: 다른 디바이스들과 동일한 패턴
4. **시뮬레이터 통합**: Omniverse와 완전 호환
5. **간단한 설정**: 추가 설정 파일 불필요

이제 8BitDo Controller가 Isaac Lab의 다른 디바이스들과 완전히 동일한 방식으로 작동합니다! 🎮🤖 