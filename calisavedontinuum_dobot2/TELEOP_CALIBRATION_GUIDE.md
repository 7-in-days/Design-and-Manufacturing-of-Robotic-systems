# Teleop Calibration Guide

이 문서는 `teleop_calibration.py`를 사용해서 continuum robot mission controller의
lookup table 값을 보정하는 방법을 정리한 가이드입니다.

## 목적

`teleop_calibration.py`는 mission controller용 lookup table을 만들기 위한 보정 도구입니다.

즉, 미션 실행 코드가 아니라 `TARGET_TABLE`과 `TRANSITION_TABLE` 값을 실험적으로 얻기 위한
teleoperation tool입니다.

## 기본 실행 흐름

```text
1. ROS2 초기화
2. ContinuumController 생성
3. Dynamixel 연결 확인
4. 현재 motor encoder position 읽기
5. 키보드 명령으로 motor position 미세 조정
6. approach / final 위치 저장
7. calibration.py에 넣을 snippet 출력
8. 종료 시 torque off 및 rclpy shutdown
```

## 핵심 구조

```python
controller = ContinuumController()
controller.check_connecting()
```

이 부분에서 실제 모터와 연결합니다.

따라서 이 코드는 시뮬레이션용이 아니라 실제 Dynamixel과 통신하는 코드입니다.

## 조작 방식

```text
q / a : motor 0 position 증가 / 감소
w / s : motor 1 position 증가 / 감소
z / x : 두 motor position 동시에 증가 / 감소
+ / - : 이동 step 크기 조절
p     : 현재 feedback 출력
g     : motor position 직접 입력
h     : 현재 위치를 approach로 저장
f     : 현재 위치를 final로 저장
o     : TARGET_TABLE 형식 출력
t     : TRANSITION_TABLE 형식 출력
exit  : 종료
```

## TARGET_TABLE 보정 절차

`TARGET_TABLE`은 target 1, 2, 3의 기본 위치를 보정할 때 사용합니다.

```text
1. teleop_calibration 실행
2. target mode 선택
3. Target id 입력
4. q/a/w/s로 로봇 끝단을 target 근처까지 이동
5. target 근처의 안전한 접근 위치에서 h 입력
6. target에 정확히 맞춘 위치에서 f 입력
7. o 입력
8. 출력된 dict를 calibration.py의 TARGET_TABLE에 복사
```

예시 출력:

```python
1: {
    "approach": {1: 1200, 2: 2150},
    "final":    {1: 1250, 2: 2100},
    "move_time": 0.60,
    "settle_time": 0.25,
},
```

## TRANSITION_TABLE 보정 절차

`TRANSITION_TABLE`은 특정 이동 조합에서 오차가 반복될 때만 사용합니다.

예를 들어 `1 -> 2` 이동에서만 target 2가 반복적으로 빗나간다면 `(1, 2)` transition을 따로 보정합니다.

```text
1. teleop_calibration 실행
2. transition mode 선택
3. Previous target id 입력
4. Next target id 입력
5. 먼저 previous target 위치에서 시작
6. q/a/w/s로 next target에 맞도록 이동
7. approach 위치에서 h 입력
8. final 위치에서 f 입력
9. t 입력
10. 출력된 dict를 calibration.py의 TRANSITION_TABLE에 복사
```

예시 출력:

```python
(1, 2): {
    "approach": {1: 1690, 2: 1860},
    "final":    {1: 1760, 2: 1810},
    "move_time": 0.55,
    "settle_time": 0.25,
},
```

## 코드에서 중요한 함수

### `positions_from_feedback(controller)`

현재 모터 encoder position을 읽어서 `{0: pos0, 1: pos1}` 형태로 반환합니다.

### `set_positions(controller, positions)`

입력된 motor position으로 실제 모터를 이동시킵니다.

내부적으로 `controller.move_to_position_smooth()`를 사용합니다.

### `nudge_motor(...)`

특정 모터 하나만 step만큼 이동합니다.

예를 들어 `q` 명령은 motor 0의 position을 `+step`만큼 증가시킵니다.

### `nudge_all(...)`

두 모터를 동시에 step만큼 이동합니다.

### `print_target_snippet(...)`

`TARGET_TABLE`에 붙여넣을 수 있는 형태로 현재 저장된 approach/final 값을 출력합니다.

### `print_transition_snippet(...)`

`TRANSITION_TABLE`에 붙여넣을 수 있는 형태로 현재 저장된 approach/final 값을 출력합니다.

## 안전 가이드

현재 position은 XL430 기준으로 `0~4095` 사이로 제한됩니다.

```python
MIN_POSITION = 0
MAX_POSITION = 4095
```

하지만 소프트웨어 제한만으로 충분하지 않을 수 있습니다. 실제 실험에서는 다음을 지켜야 합니다.

```text
- 처음에는 step을 작게 사용
- 모터 전원 켜기 전에 와이어 장력 확인
- 로봇 끝단이 물체에 걸리지 않았는지 확인
- 이상한 소리나 과전류 느낌이 있으면 즉시 exit 또는 Ctrl+C
- transition 보정 전에는 반드시 previous target 위치에서 시작
```

## ROS2 실행 연결

`setup.py`의 `console_scripts`에 다음 entry를 추가합니다.

```python
entry_points={
    "console_scripts": [
        "mission = continuum_mission.main:main",
        "teleop_calibration = continuum_mission.teleop_calibration:main",
    ],
},
```

빌드 후 실행 예시는 다음과 같습니다.

```bash
colcon build
source install/setup.bash
ros2 run continuum_mission teleop_calibration
```

## 발표용 요약

`teleop_calibration.py`는 continuum robot의 lookup table을 실험적으로 만들기 위한
보정용 teleoperation tool입니다.

사용자는 키보드로 Dynamixel motor position을 미세 조정한 뒤 현재 encoder 값을
`TARGET_TABLE` 또는 `TRANSITION_TABLE` 형식으로 출력할 수 있습니다.
