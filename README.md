# ROS 2 자율주행 레이싱 시뮬레이션

레이싱 시뮬레이터를 기반으로 경로 추종, LiDAR 장애물 인식, 회피 판단, 속도 제어를 통합한 ROS 2 프로젝트입니다.

레이싱 경로와 속도 프로파일을 활용하고, 장애물 판단 결과를 Pure Pursuit 제어에 반영하도록 구성했습니다.

## 주요 기능

- **차량 시뮬레이션**: 차량 상태와 가상 LiDAR 데이터 생성
- **레이싱 경로 제공**: CSV 기반 경로, 좌우 경계, 속도 프로파일 발행
- **경로 추종**: Pure Pursuit 기반 조향 제어
- **가변 Lookahead**: 속도와 경로 곡률에 따른 목표점 거리 조정
- **장애물 인식**: LiDAR DBSCAN 클러스터링
- **회피 판단**: 장애물과 주행 경계를 고려한 횡방향 오프셋·조향 바이어스·감속 계수 계산
- **주행 복귀**: 회피 후 기준 경로로 돌아가는 상태 관리
- **시각화**: RViz에서 지도, 경로 및 차량 상태 확인

## 시스템 구조

가상 LiDAR → DBSCAN 인식 → 회피 판단 → Pure Pursuit 제어 → 차량 시뮬레이터

레이싱 경로 발행 노드는 기준 경로, 좌우 경계, 목표 속도 정보를 제어 노드에 제공합니다.

## 기술적 구현

### 곡률과 속도를 고려한 경로 추종

Pure Pursuit의 Lookahead를 속도와 경로 곡률에 따라 조정합니다. 곡률 기반 피드포워드와 yaw rate 관련 보정을 조향 계산에 반영합니다.

### 경로 기반 속도 프로파일

오프라인 스크립트에서 경로의 위치, 곡률, 목표 속도, 조향 및 경계 정보를 생성합니다. 횡가속도와 가속·감속 제한을 고려해 구간별 속도를 계산합니다.

### 인식·판단·제어 분리

장애물 인식 결과와 회피 판단 결과를 ROS 메시지로 전달합니다. 회피 판단 노드는 횡방향 오프셋, 조향 바이어스, 속도 계수를 계산하고 제어 노드는 이를 주행 명령에 반영합니다.

### 장애물 기억과 출력 필터링

최근 장애물 위치를 일정 시간 유지하는 메모리 로직과 1차 필터를 사용합니다. 회피 판단에는 주행 경계 여유와 장애물 위치를 반영합니다.

## 기술 스택

- C++17, Python
- ROS 2, rclcpp
- ament_cmake, colcon
- RViz2
- Pure Pursuit
- DBSCAN
- Ackermann 차량 제어
- CSV 기반 경로 및 속도 프로파일 처리

## 패키지 구성

| 디렉터리 | ROS 패키지 | 역할 |
| --- | --- | --- |
| `src/racecar_simulator` | `racecar_simulator` | 차량·LiDAR 시뮬레이션, 지도, 주행 통계 |
| `src/pure_pursuit` | `pure_pursuit` | 경로 발행 및 추종 제어 |
| `src/perception` | `perception_dbscan` | LiDAR 장애물 클러스터링 |
| `src/decision` | `decision_ftg` | 회피 방향 및 감속 판단 |
| `src/control_msgs` | `control_msgs` | 제어 관련 서비스 인터페이스 |

## 주요 파일

| 파일 | 역할 |
| --- | --- |
| `src/pure_pursuit/src/pure_pursuit.cpp` | 경로 추종 및 차량 제어 |
| `src/pure_pursuit/src/racing_line_publisher.cpp` | 경로·경계·속도 프로파일 발행 |
| `src/perception/src/perception_dbscan_node.cpp` | 장애물 인식 |
| `src/decision/src/decision_ftg_node.cpp` | 회피 및 감속 판단 |
| `src/pure_pursuit/scripts/generate_iccas2025_racing_profile.py` | 레이싱 프로파일 생성 |
| `src/racecar_simulator/launch/simulator.launch.py` | 통합 실행 |

## 빌드 및 실행

1. `simulator.launch.py`의 `pkg_dir`을 저장소 내 시뮬레이터 패키지 경로로 수정합니다.
2. 선택한 지도에 필요한 지도·경로·프로파일 파일을 확인합니다.
3. ROS 2 환경에서 워크스페이스를 빌드하고 실행합니다.

    colcon build --symlink-install
    source install/setup.bash
    ros2 launch racecar_simulator simulator.launch.py

## 프로젝트에서 다룬 역량

- ROS 2 기반 인식·판단·제어 파이프라인 구성
- 차량 경로 추종 및 속도 계획
- LiDAR 클러스터링과 장애물 회피 로직
- 상태 머신 및 필터 기반 제어 출력 관리
- 시뮬레이션을 활용한 알고리즘 개발

## 외부 리소스

저장소에는 F1TENTH 트랙 지도 리소스가 포함되어 있습니다. 해당 리소스의 출처와 라이선스는 `src/racecar_simulator/maps/f1tenth_racetracks`의 README 및 LICENSE를 참고합니다.
