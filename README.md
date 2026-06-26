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

## 코드 흐름 그래프 보는 법 (Obsidian Graph)

각 브랜치의 `code-flow/` 폴더는 **Obsidian Vault**입니다. 모듈 하나당 노트 하나로 구성되어 있고
`[[wikilink]]`로 호출·의존 관계를 연결해 두었습니다.

1. [Obsidian](https://obsidian.md) 설치 후 **Open folder as vault**로 해당 브랜치의 `code-flow/` 폴더를 엽니다.
2. 좌측의 **Graph View**(Ctrl/Cmd+G)를 켜면 코드 흐름이 그래프로 시각화됩니다.

GitHub에서 바로 볼 수 있도록 각 README에는 동일한 흐름을 **Mermaid** 다이어그램으로도 넣어 두었습니다.
