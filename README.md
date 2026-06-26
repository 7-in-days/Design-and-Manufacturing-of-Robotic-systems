# Design and Manufacturing of Robotic Systems

2026학년도 1학기 **Design and Manufacturing of Robotic systems** 텀 프로젝트 저장소입니다.

이 저장소는 두 개의 텀 프로젝트(4족 보행 로봇 Go2 주행, Continuum Robot 제어/SMA 브레이크)를
주제별로 **브랜치(branch)** 로 나누어 관리합니다. `main` 브랜치는 안내용이며, 실제 코드와
보고서는 아래 각 브랜치에 있습니다.

## 브랜치 구성

| 브랜치 | 프로젝트 | 내용 |
|--------|----------|------|
| [`go2-sim`](../../tree/go2-sim) | Project 1 | Unitree Go2 **시뮬레이션(Isaac Sim)** DWA 주행 |
| [`go2-real`](../../tree/go2-real) | Project 1 | Unitree Go2 **실물(Real)** DWA 주행 |
| [`continumm`](../../tree/continumm) | Project 2 | Tendon-driven **Continuum Robot** 제어 + SMA 브레이크 (+ legacy) |

각 브랜치의 `README.md`에는 다음이 포함되어 있습니다.

1. 해당 프로젝트 최종 보고서 내용 (서론 / 본론 / 결과 / 결론)
2. **코드 흐름 그래프** — GitHub용 Mermaid 다이어그램 + Obsidian Graph View용 노트 모음(`code-flow/`)
3. 코드 흐름에 대한 설명

## 프로젝트 요약

### Project 1 — Unitree Go2 Sim-to-Real 주행 (`go2-sim`, `go2-real`)
`/occupancy_map` 토픽을 입력으로 받아 **DWA(Dynamic Window Approach)** 기반 local planner로
장애물을 회피하며 `/cmd_vel`을 발행한다. 동일한 DWA 구조를 시뮬레이션과 실물에 적용하여
**Sim-to-Real Gap**(센서 입력 안정성, 좌표계 정의, actuator response 등)을 분석한다.

### Project 2 — Continuum Robot 제어 / SMA 브레이크 (`continumm`)
끝단 레이저를 표적에 정렬하기 위해 **target별 calibration lookup table** 기반의 개루프 제어를
수행하고, **minimum-jerk** 보간으로 모터 위치를 부드럽게 잇는다. 형상 고정을 위한
**SMA(Shape Memory Alloy) 브레이크**(유성기어 구조)를 설계·실험한다.

## 미션 개요 (무엇을 하는 과제인가)

수업에서 제시된 두 텀 프로젝트의 과제 내용은 다음과 같다.

### Project 1 — Go2 주행 & Path Planning (`go2-sim`, `go2-real`)

4족 보행 로봇 **Unitree Go2**를 **Simulation(NVIDIA Isaac Sim)** 과 **실제(Real)** 환경에서
운용하며, 주어진 코스를 따라 **장애물(트래픽 콘)을 피해 자율 주행**하도록 path planning을
구현하는 과제이다.

- 서로 다른 **두 종류의 코스**를 주행한다.
- 주행 판단의 입력으로 LiDAR로부터 생성된 **2D Occupancy Map**을 사용할 수 있다.
  - 400 × 400 cell, 해상도 0.03 m (약 12 m × 12 m), 로봇 중심은 맵 중앙(예: `[200, 200]`).
  - cell 값: 벽·장애물 = 100, 빈 공간·미확인 = 0/−1. (Simulation과 Real LiDAR 동일 규격)
  - LiDAR 사용 여부는 선택 사항이다.
- Simulation용 코드와 Real용 코드는 **동일하게 써도 되고 다르게 써도 된다.** (이 저장소에서는
  두 환경의 차이를 분석하기 위해 `go2-sim` / `go2-real`로 나누어 구현하였다.)

### Project 2 — Continuum Robot 제어 & SMA 브레이크 (`continumm`)

유연한 몸체를 가진 **tendon(wire) 구동 Continuum Robot**을 다루는 과제로, 두 부분으로 구성된다.

1. **Continuum 제어 미션** — Dynamixel 모터로 wire를 구동하여 로봇을 휘게 하고, **끝단에 부착된
   레이저 포인터를 주어진 표적(target)에 정렬**시킨다. 좁고 굽은 공간 접근이 가능한 continuum
   robot의 특성을 이용한 정밀 조준 과제이다.
2. **SMA 브레이크 설계 미션** — Continuum robot은 wire 장력만으로는 외력·하중에 자세가 흐트러지므로,
   **형상을 고정(lock)하고 wire에 걸리는 하중을 견디는 브레이크**가 필요하다. **SMA(형상기억합금)** 를
   이용해 브레이크 구조를 **창의적으로 설계하고 3D 프린팅으로 제작**한다.
   - 주요 설계 조건: 중심에 대해 대칭 구조, 크기 10 cm × 10 cm × 5 cm 이내, 제공되는 비즈 와이어
     (1.5 mm 볼)와 SMA(동일 길이 10코일 이내) 사용, UTM에 고정 가능한 형상.
   - 동작 조건: 브레이크 OFF 시 자유 이동 → ON 시 형상 고정 → 다시 OFF 후 (냉각 포함) 약 1분 이내에
     다시 자유롭게 움직일 수 있어야 한다.

> 이 절은 과제가 **무엇을 하는지**만 정리한 것으로, 평가·채점 방식은 포함하지 않는다.

## 코드 흐름 그래프 보는 법 (Obsidian Graph)

각 브랜치의 `code-flow/` 폴더는 **Obsidian Vault**입니다. 모듈 하나당 노트 하나로 구성되어 있고
`[[wikilink]]`로 호출·의존 관계를 연결해 두었습니다.

1. [Obsidian](https://obsidian.md) 설치 후 **Open folder as vault**로 해당 브랜치의 `code-flow/` 폴더를 엽니다.
2. 좌측의 **Graph View**(Ctrl/Cmd+G)를 켜면 코드 흐름이 그래프로 시각화됩니다.

GitHub에서 바로 볼 수 있도록 각 README에는 동일한 흐름을 **Mermaid** 다이어그램으로도 넣어 두었습니다.
