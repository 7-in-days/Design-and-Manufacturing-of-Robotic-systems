# Calibration Table Guide

이 문서는 `calibration.py`의 `TARGET_TABLE`과 `TRANSITION_TABLE`을 어떻게 이해하고,
어떻게 보정해서 사용하는지 설명합니다.

## 핵심 개념

`calibration.py`는 continuum robot mission controller가 사용할 lookup table을 저장하는 파일입니다.

현재 코드는 카메라 좌표나 Cartesian 좌표를 직접 계산하지 않습니다. 대신 미리 실험적으로 구한
motor encoder position을 table에 저장해두고, mission 실행 중 target 번호에 맞는 motor position을
꺼내서 사용합니다.

즉 구조는 다음과 같습니다.

```text
target 번호
    ↓
calibration.py lookup table
    ↓
motor 1, motor 2 encoder position
    ↓
Dynamixel position control
```

## Target 번호와 Motor 번호 구분

`TARGET_TABLE`의 바깥쪽 key는 target 번호입니다.

```python
TARGET_TABLE[1]
```

이 뜻은 `1번 모터`가 아니라 `1번 target`입니다.

각 target 안쪽의 dict에서 `1`, `2`가 motor id입니다.

```python
"approach": {1: 1200, 2: 2150}
```

이 뜻은 다음과 같습니다.

```text
motor 1 position = 1200
motor 2 position = 2150
```

따라서 현재 구조는 모터 1개 기준이 아니라, 모터 2개의 position pair를 저장하는 구조입니다.

## TARGET_TABLE

`TARGET_TABLE`은 각 target에 대한 기본 이동 정보를 저장합니다.

```python
TARGET_TABLE = {
    1: {
        "approach": {1: 1200, 2: 2150},
        "final":    {1: 1250, 2: 2100},
        "move_time": 0.6,
        "settle_time": 0.25,
    },
}
```

각 필드의 의미는 다음과 같습니다.

```text
approach   : target에 바로 닿기 전의 안전한 접근 위치
final      : 실제 target에 맞춘 최종 위치
move_time  : approach + final 이동에 사용할 총 이동 시간
settle_time: final 위치에서 안정화됐다고 판단하기 위해 기다리는 시간
```

mission 실행 중 첫 target으로 이동하거나, 특정 transition 보정값이 없을 때는
`TARGET_TABLE[next_target]`을 사용합니다.

## 왜 approach와 final을 나누는가

continuum robot은 와이어 장력, 마찰, 탄성 때문에 한 번에 target final 위치로 이동하면
끝단이 흔들리거나 overshoot할 수 있습니다.

그래서 현재 mission 코드는 한 target 이동을 두 단계로 나눕니다.

```text
1. approach 위치로 이동
2. final 위치로 이동
3. stable 확인
4. hold time 동안 유지
```

`mission.py`에서는 `move_time`을 다음 비율로 나눕니다.

```text
approach move: move_time의 60%
final move   : move_time의 40%
```

예를 들어 `move_time = 0.6`이면:

```text
approach_time = 0.36 s
final_time    = 0.24 s
```

## TRANSITION_TABLE

`TRANSITION_TABLE`은 이전 target에서 다음 target으로 이동할 때만 적용하는 보정 table입니다.

```python
TRANSITION_TABLE = {
    (1, 2): {
        "approach": {1: 1690, 2: 1860},
        "final":    {1: 1760, 2: 1810},
        "move_time": 0.55,
        "settle_time": 0.25,
    },
}
```

여기서 `(1, 2)`는 motor 1, motor 2가 아니라 다음 의미입니다.

```text
previous target = 1
next target     = 2
```

즉 `target 1에서 target 2로 이동할 때`만 이 보정값을 사용합니다.

안쪽의 `{1: 1690, 2: 1860}`은 motor position입니다.

```text
motor 1 position = 1690
motor 2 position = 1860
```

## 왜 TRANSITION_TABLE이 필요한가

continuum robot은 같은 target으로 가더라도 이전 자세에 따라 실제 끝단 위치가 달라질 수 있습니다.

예를 들어:

```text
target 1 -> target 2
target 3 -> target 2
```

둘 다 최종 목표는 target 2이지만, 출발 자세가 다르기 때문에 와이어 slack, 마찰, 탄성 이력 때문에
실제 도착 위치가 다를 수 있습니다.

이런 현상을 보정하기 위해 `TRANSITION_TABLE`을 사용합니다.

중요한 점은 모든 transition을 처음부터 다 채울 필요가 없다는 것입니다.

```text
- 기본적으로 TARGET_TABLE 사용
- 특정 transition에서 반복적으로 오차가 생기면 TRANSITION_TABLE에 추가
- TRANSITION_TABLE에 없는 조합은 TARGET_TABLE로 fallback
```

## mission.py에서 table을 사용하는 방식

`mission.py`의 `get_motion()` 함수가 어떤 table을 사용할지 결정합니다.

```python
def get_motion(prev_target, next_target):
    if prev_target is None:
        return TARGET_TABLE[next_target]

    key = (prev_target, next_target)
    if key in TRANSITION_TABLE:
        return TRANSITION_TABLE[key]
    else:
        return TARGET_TABLE[next_target]
```

사용 우선순위는 다음과 같습니다.

```text
1. 첫 target이면 TARGET_TABLE 사용
2. (prev_target, next_target)이 TRANSITION_TABLE에 있으면 TRANSITION_TABLE 사용
3. 없으면 TARGET_TABLE[next_target] 사용
```

## 보정 Workflow

### 1. TARGET_TABLE 보정

`teleop_calibration.py`를 사용해서 target별 기본 위치를 얻습니다.

```text
1. teleop_calibration 실행
2. target mode 선택
3. Target id 입력
4. q/a/w/s로 motor 1, motor 2를 조절
5. target 근처의 approach 위치에서 h 입력
6. 정확히 target에 맞는 final 위치에서 f 입력
7. o 입력
8. 출력된 snippet을 TARGET_TABLE에 복사
```

### 2. TRANSITION_TABLE 보정

특정 transition에서만 오차가 반복될 때 보정합니다.

```text
1. teleop_calibration 실행
2. transition mode 선택
3. previous target id 입력
4. next target id 입력
5. previous target 위치에서 시작
6. next target으로 이동하도록 q/a/w/s 조절
7. approach 위치에서 h 입력
8. final 위치에서 f 입력
9. t 입력
10. 출력된 snippet을 TRANSITION_TABLE에 복사
```

## 예시: Scenario 1-2-1

scenario가 다음과 같다고 가정합니다.

```python
SCENARIO_TEXT = "1-2-1"
```

실행 중 table 선택은 다음과 같습니다.

```text
첫 번째 target 1:
prev_target = None
TARGET_TABLE[1] 사용

두 번째 target 2:
prev_target = 1
TRANSITION_TABLE[(1, 2)]가 있으면 사용
없으면 TARGET_TABLE[2] 사용

세 번째 target 1:
prev_target = 2
TRANSITION_TABLE[(2, 1)]가 있으면 사용
없으면 TARGET_TABLE[1] 사용
```

## 발표용 요약

`calibration.py`는 target 번호를 실제 Dynamixel motor command position으로 변환하는 lookup table입니다.

`TARGET_TABLE`은 target별 기본 motor position을 저장하고, `TRANSITION_TABLE`은 이전 target에 따라
달라지는 continuum robot의 이력 의존성, slack, 마찰, 탄성 효과를 보정하기 위한 경험적 table입니다.

현재 코드는 좌표 기반 제어가 아니라, 실험적으로 얻은 motor 1, motor 2의 encoder position pair를
사용하는 open-loop mission controller 구조입니다.
