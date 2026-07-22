# Malaga Ouster LIO-SAM + NDT 프로젝트 전체 가이드

## 1. 프로젝트 목적

이 프로젝트는 Malaga 데이터셋의 Ouster LiDAR/IMU rosbag을 사용해 다음 두 단계를 수행한다.

1. **LIO-SAM mapping**: LiDAR와 IMU로 차량 궤적을 추정하고 전역 PCD 지도를 만든다.
2. **NDT localization**: 저장된 PCD 지도와 현재 LiDAR scan을 정합해 지도 안의 현재 위치를 구한다.

LIO-SAM은 **지도 제작**, NDT는 **완성된 지도 위에서 위치 추정**을 담당한다. 따라서 기본
실행 순서는 `LIO-SAM 실행 → 지도 저장 → LIO-SAM 종료 → NDT 실행`이다.

## 2. 전체 데이터 흐름

```text
Malaga etsii rosbag                  Ouster metadata rosbag
├── /ouster/range_image             └── /ouster/metadata (센서 JSON)
└── /ouster/imu                                  │
            │                                    │
            └────────────┬───────────────────────┘
                         ▼
              ouster_range_to_cloud.py
        range pixel + 빔 각도 → XYZ, ring, t 복원
                         │
                         └── /ouster/points (PointCloud2)
                                      │
                 ┌────────────────────┴────────────────────┐
                 │                                         │
                 ▼                                         ▼
          LIO-SAM mapping                          NDT localization
  IMU 적분 → deskew → 특징 추출          저장 PCD(target) + 현재 scan(source)
  → scan-to-map → factor graph                     → pose 최적화
  → loop closure → 지도                           → TF/odom/path
                 │                                         │
                 ▼                                         ▼
          GlobalMap.pcd                           /ndt/odometry
                                                 map → os_lidar TF
```

## 3. 실제 디렉터리 구조

```text
/home/dz/my_autonomous_vehicle/
├── src/
│   ├── LIO-SAM/
│   │   ├── launch/
│   │   │   ├── malaga_mapping.launch.py       # mapping 전체 실행
│   │   │   ├── ndt_localization.launch.py     # NDT node 실행
│   │   │   └── run.launch.py                  # 원본 LIO-SAM 실행
│   │   ├── config/
│   │   │   ├── malaga_os1.yaml                # Malaga/LIO-SAM 파라미터
│   │   │   ├── ndt_localization.yaml          # NDT 파라미터
│   │   │   └── rviz2.rviz                     # mapping RViz 설정
│   │   ├── scripts/
│   │   │   └── ouster_range_to_cloud.py       # range image → PointCloud2
│   │   ├── src/
│   │   │   ├── imuPreintegration.cpp          # IMU 적분과 고주파 odometry
│   │   │   ├── imageProjection.cpp            # scan 구성과 motion deskew
│   │   │   ├── featureExtraction.cpp          # edge/surface 특징 추출
│   │   │   ├── mapOptmization.cpp             # factor graph/map/loop closure
│   │   │   └── ndtLocalization.cpp            # PCL NDT 위치 추정
│   │   ├── msg/CloudInfo.msg                   # LIO-SAM node 간 데이터 묶음
│   │   ├── srv/SaveMap.srv                     # PCD 저장 서비스
│   │   ├── CMakeLists.txt                      # 실행 파일 빌드 정의
│   │   └── package.xml                         # ROS 2 dependency
│   └── my_vehicle_description/                 # 차량 URDF/Xacro
├── build/                                      # colcon 중간 산출물
├── install/                                    # source 후 사용하는 실행 환경
├── log/                                        # colcon build 로그
└── PROJECT_FLOW_GUIDE.md                       # 현재 문서
```

직접 수정할 코드는 `src/` 아래에 있다. `build/`, `install/`, `log/`는 colcon이 생성하므로
일반적으로 직접 수정하지 않는다.

## 4. 입력 데이터와 변환기

### 4.1 입력 rosbag

기본 데이터 경로는 다음과 같다.

```text
/home/dz/datasets/malaga_os1/etsii_rosbag/etsii_rosbag
/home/dz/datasets/malaga_os1/metadata_rosbag/metadata_rosbag
```

ETSII bag에는 완성된 `/ouster/points`가 없고 `/ouster/range_image`가 있다. 별도 metadata
bag에는 Ouster 모델, 빔 각도, 수평 해상도, FPS가 포함된 JSON이 있다.

### 4.2 `ouster_range_to_cloud.py`

변환 순서는 다음과 같다.

1. metadata bag의 `/ouster/metadata` 문자열을 읽는다.
2. `ouster.sdk.client.SensorInfo`로 센서 구조를 해석한다.
3. range image 값에 `4`를 곱해 데이터셋 단위(4 mm)를 mm로 복원한다.
4. `destagger(..., inverse=True)`로 Ouster native firing 순서를 복원한다.
5. `XYZLut`로 각 range pixel을 XYZ 점으로 변환한다.
6. range가 0인 invalid point를 제거한다.
7. LIO-SAM deskew에 필요한 `ring`과 frame 내부 상대시간 `t`를 넣는다.
8. `/ouster/points`를 `sensor_msgs/msg/PointCloud2`로 발행한다.

핵심은 단순 XYZ cloud가 아니라 `x, y, z, intensity, t, reflectivity, ring, noise, range`
필드 구조를 만든다는 점이다. 특히 `ring`과 `t`가 없으면 LIO-SAM의 scan deskew가 정상적으로
동작하지 않는다.

## 5. LIO-SAM 코드 흐름

### 5.1 `imuPreintegration.cpp`

- `/ouster/imu`를 고주파로 받는다.
- GTSAM IMU preintegration으로 짧은 시간의 이동을 예측한다.
- LiDAR mapping 결과가 들어오면 IMU bias와 상태를 다시 보정한다.
- `/lio_sam/odometry/imu_incremental`과 IMU path를 발행한다.

IMU는 LiDAR scan 사이의 빠른 움직임을 예측하고, LiDAR 결과는 시간이 지나며 생기는 IMU
drift를 억제한다.

코드 안에서는 용도가 다른 IMU 적분기 두 개를 유지한다. 최적화용 적분기는 LiDAR 보정 시각
사이의 factor를 만들고, 실시간용 적분기는 최신 bias로 남은 IMU를 재적분해 고주파 odometry를
만든다. TransformFusion은 마지막 LiDAR pose에 그 이후의 IMU 상대 이동만 합성한다.

### 5.2 `imageProjection.cpp`

- `/ouster/points`, `/ouster/imu`, incremental odometry를 시간 동기화한다.
- organized range image 형태로 점을 배치한다.
- 한 scan을 취득하는 동안 차량이 움직여 생긴 휘어짐을 `ring/t + IMU`로 deskew한다.
- 유효 점과 부가정보를 `CloudInfo`로 묶어 다음 node로 전달한다.

주요 출력은 `/lio_sam/deskew/cloud_deskewed`와 `/lio_sam/deskew/cloud_info`다.

### 5.3 `featureExtraction.cpp`

- 각 scan line의 곡률을 계산한다.
- 곡률이 큰 점을 corner/edge 특징으로 선택한다.
- 곡률이 작은 점을 surface 특징으로 선택한다.
- 가려진 점과 불안정한 점을 제외하고 voxel downsampling한다.

주요 출력은 `/lio_sam/feature/cloud_corner`, `/lio_sam/feature/cloud_surface`, 그리고 특징점이
포함된 `/lio_sam/feature/cloud_info`다.

### 5.4 `mapOptmization.cpp`

- 현재 특징점 scan을 주변 keyframe map에 맞추는 scan-to-map 최적화를 수행한다.
- 일정 거리/각도 이상 움직였을 때 새 keyframe을 만든다.
- GTSAM iSAM2 factor graph에 odometry, GPS(있는 경우), loop factor를 추가한다.
- 과거 위치 근처로 돌아오면 ICP 기반 loop closure를 검사해 누적 drift를 보정한다.
- trajectory, local/global map, odometry, path를 발행한다.
- `/lio_sam/save_map` 서비스 요청을 받으면 PCD를 저장한다.

즉 LIO-SAM 내부 흐름은 다음과 같다.

```text
IMU prediction
   ↓
PointCloud deskew
   ↓
edge/surface feature extraction
   ↓
scan-to-map optimization
   ↓
keyframe + factor graph update
   ↓
loop closure correction
   ↓
trajectory + global PCD map
```

### 5.5 소스 주석과 긴 파일 읽는 방법

다섯 C++ 파일에는 파일 전체 역할과 주요 처리 단계에 대한 한국어 주석이 들어 있다. 특히
`mapOptmization.cpp`는 기능이 한 파일에 모여 있어 길지만 다음 순서로 나눠 읽을 수 있다.

1. `laserCloudInfoHandler`: mapping 메인 루프와 실행 주기 제한
2. `extractSurroundingKeyFrames` / `scan2MapOptimization`: local map과 scan 정합
3. `saveKeyFramesAndFactor`: odometry/GPS/loop factor 및 keyframe 저장
4. `performLoopClosure` / `correctPoses`: loop 검출과 과거 pose 전역 보정
5. `publishOdometry` / `publishFrames`: odometry, TF, 지도 및 확인용 cloud 발행

현재 구조는 원본 LIO-SAM의 클래스 경계를 유지한다. 파일을 억지로 짧게 나누면 공유 상태와
thread synchronization까지 함께 바꿔야 하므로, 우선 단계 주석으로 탐색성을 높이고 계산 결과를
바꾸지 않는 경량화만 적용했다.

## 6. Malaga mapping launch가 실행하는 프로세스

`malaga_mapping.launch.py`는 다음 순서로 node를 구성한다.

| 순서 | 실행 항목 | 역할 |
|---:|---|---|
| 1 | static TF `map → odom` | mapping 결과의 frame 연결 |
| 2 | `ouster_range_to_cloud.py` | range image를 cloud로 복원 |
| 3 | `lio_sam_imuPreintegration` | IMU 기반 움직임 예측 |
| 4 | `lio_sam_imageProjection` | scan deskew/투영 |
| 5 | `lio_sam_featureExtraction` | 특징점 추출 |
| 6 | `lio_sam_mapOptimization` | 지도 및 trajectory 최적화 |
| 7 | RViz2 | 결과 시각화 |
| 8 | 3초 지연 후 bag play | subscriber 준비 뒤 데이터 재생 |

bag은 `--clock --rate 0.5`로 재생한다. `use_sim_time: true`인 모든 node가 실제 PC 시간 대신
bag의 `/clock`을 사용하며, 0.5배 재생은 CPU 처리 누락 가능성을 낮춘다.

## 7. 주요 LIO-SAM 파라미터

설정 파일: `src/LIO-SAM/config/malaga_os1.yaml`

| 파라미터 | 현재 값 | 의미 |
|---|---:|---|
| `N_SCAN` | 32 | OS1 수직 channel 수 |
| `Horizon_SCAN` | 1024 | 한 frame 수평 column 수 |
| `lidarMinRange` / `lidarMaxRange` | 1 / 120 m | 사용할 거리 범위 |
| `extrinsicRot`, `extrinsicRPY` | X/Y 축 -1 | IMU-LiDAR 장착축 회전 보정 |
| `odometrySurfLeafSize` | 0.4 m | odometry surface voxel 크기 |
| `mappingCornerLeafSize` | 0.2 m | mapping edge voxel 크기 |
| `mappingSurfLeafSize` | 0.4 m | mapping surface voxel 크기 |
| `mappingProcessInterval` | 0.2 s | mapping 계산 간격 |
| `surroundingkeyframeAddingDistThreshold` | 1.0 m | keyframe 최소 이동 |
| `loopClosureEnableFlag` | true | loop closure 사용 여부 |
| `historyKeyframeFitnessScore` | 0.3 | loop ICP 허용 기준 |

extrinsic, IMU noise, scan 구조는 센서에 맞지 않으면 지도가 크게 틀어진다. 속도 튜닝은 보통
voxel 크기, `mappingProcessInterval`, keyframe 간격 순으로 검토한다.

## 8. 지도 생성 실행 방법

### 8.1 빌드

```bash
cd /home/dz/my_autonomous_vehicle
source /opt/ros/humble/setup.bash
colcon build --packages-select lio_sam --symlink-install
source install/setup.bash
```

### 8.2 mapping 실행

```bash
cd /home/dz/my_autonomous_vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lio_sam malaga_mapping.launch.py
```

다른 bag 또는 속도를 지정하려면 다음처럼 launch argument를 사용한다.

```bash
ros2 launch lio_sam malaga_mapping.launch.py \
  bag_path:=/path/to/etsii_bag \
  metadata_bag:=/path/to/metadata_bag \
  rate:=0.25
```

### 8.3 지도 저장

다른 터미널에서 다음을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/dz/my_autonomous_vehicle/install/setup.bash
mkdir -p /home/dz/maps/malaga

ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.0, destination: '/maps/malaga'}"
```

이 LIO-SAM 구현은 `destination` 앞에 사용자의 home 경로를 자동으로 붙인다. 따라서
`/home/dz/maps/malaga`를 만들려면 service에는 `/maps/malaga`를 전달해야 한다.

저장 후 `/home/dz/maps/malaga/GlobalMap.pcd`가 만들어졌는지 확인한다.

```bash
ls -lh /home/dz/maps/malaga/GlobalMap.pcd
```

## 9. NDT 알고리즘과 이 프로젝트의 구현

NDT(Normal Distributions Transform)는 target point cloud 공간을 일정한 voxel로 나누고,
각 voxel 안의 점들을 평균과 공분산을 가진 정규분포로 표현한다. 현재 scan의 점들이 이
분포에서 높은 확률을 갖도록 6-DoF 변환 `(x, y, z, roll, pitch, yaw)`을 반복 최적화한다.

이 프로젝트의 `ndtLocalization.cpp` 처리 순서는 다음과 같다.

1. 시작 시 PCD 지도를 한 번 로드한다.
2. map voxel filter를 적용하고 NDT의 고정 target으로 설정한다.
3. `/ouster/points`를 받을 때마다 scan voxel filter를 적용한다.
4. `현재 pose × 직전 상대이동(delta)`로 다음 pose의 초기 guess를 만든다.
5. `ndt.align(aligned, guess)`로 scan(source)을 map(target)에 정합한다.
6. 수렴 여부와 fitness score를 검사한다. 단, 최초 수렴 결과는 초기 lock 형성을 위해 fitness
   상한을 적용하지 않고 이후 scan부터 상한을 적용한다.
7. 성공한 결과만 pose/delta에 반영한다.
8. odometry, path, aligned cloud와 `map → os_lidar` TF를 발행한다.

```text
PCD map ── voxel filter ──> NDT target (시작 시 1회)
                                      ▲
                                      │ align(source, initial guess)
/ouster/points ─ voxel filter ─> NDT source
                                      │
 previous pose + delta ───────────────┘
                                      │
                 converged && (first lock || fitness <= threshold)?
                         ├── No  → 결과 거부, 이전 pose 유지
                         └── Yes → pose/TF/odom/path 갱신
```

NDT는 전역 위치를 처음부터 찾는 알고리즘이 아니라 local optimizer다. 초기 pose가 실제 위치와
너무 멀면 잘못된 구조에 수렴하거나 결과가 거부될 수 있으므로 RViz의 **2D Pose Estimate**로
가까운 초기 위치를 제공해야 한다.

## 10. NDT 입출력

| 구분 | 이름 | 설명 |
|---|---|---|
| 입력 | `map_path` | LIO-SAM으로 저장한 PCD |
| 입력 | `/ouster/points` | 현재 LiDAR scan |
| 입력 | `/initialpose` | RViz에서 지정한 초기 위치 |
| 출력 | `/ndt/map` | downsample된 target map |
| 출력 | `/ndt/aligned_points` | map frame에 정합된 현재 scan |
| 출력 | `/ndt/odometry` | 추정 pose; covariance 대각에 fitness 저장 |
| 출력 | `/ndt/path` | 받아들인 NDT pose 누적 경로 |
| 출력 | TF `map → os_lidar` | 센서의 현재 지도 좌표 |

`/ndt/odometry`의 covariance 값은 실제 통계 covariance가 아니라 상태 확인용 fitness 값이다.
작을수록 scan과 map의 평균적인 거리가 가깝다.

## 11. 주요 NDT 파라미터

설정 파일: `src/LIO-SAM/config/ndt_localization.yaml`

| 파라미터 | 현재 값 | 영향 |
|---|---:|---|
| `map_leaf_size` | 0.25 m | target map 점 수/해상도 |
| `scan_leaf_size` | 0.5 m | scan 계산량/해상도 |
| `transformation_epsilon` | 0.01 | 수렴 판정 정밀도 |
| `step_size` | 0.1 | 한 최적화 step의 최대 이동 |
| `resolution` | 1.0 m | NDT 정규분포 grid 크기 |
| `maximum_iterations` | 35 | scan당 최대 반복 횟수 |
| `max_fitness_score` | 10.0 | 첫 lock 이후 이보다 나쁜 결과 거부 |
| `initial_x/y/z/yaw` | 0 | 시작 pose |

처리가 느리면 먼저 `scan_leaf_size`를 0.7~1.0으로 키운다. 초기 오차에 민감하면
`resolution`을 키워 시험하되 정밀도가 낮아질 수 있다. 파라미터는 한 번에 하나씩 바꾸고
동일한 bag 구간에서 fitness, 처리 속도, RViz 겹침을 비교한다.

실제 `map_leaf_size`는 `min(설정값, resolution × 0.25)`로 제한된다. NDT cell마다 covariance를
계산할 충분한 target point를 남겨 PCL 내부 KD-tree가 비는 문제를 피하기 위한 안전장치다.
지도와 scan의 NaN/Inf 점도 NDT 입력 전에 제거한다.

### 11.1 적용된 안전한 경량화

- NDT aligned cloud는 `/ndt/aligned_points` 구독자가 있을 때만 PointCloud2로 직렬화한다.
- NDT path는 `max_path_points`까지만 보관해 장시간 실행 시 메모리 증가를 제한한다.
- mapping은 `mappingProcessInterval`과 주변 keyframe만 사용해 매 scan의 계산량을 제한한다.
- IMU-LiDAR fusion queue가 비었을 때 다음 메시지를 기다려 잘못된 queue 접근을 방지한다.

voxel 크기나 특징점 임계값을 더 공격적으로 바꾸면 CPU 사용량은 줄지만 지도 품질도 달라질 수
있으므로 이번 코드 정리에는 포함하지 않았다.

## 12. NDT 실행 방법

먼저 mapping을 종료해 LIO-SAM과 NDT가 같은 TF를 동시에 발행하지 않도록 한다.

터미널 1에서 bag과 cloud 변환기를 실행한다. 가장 간단한 방법은 mapping launch를 NDT 전용으로
분리하는 것이지만, 현재 구성에서는 변환기와 bag player를 각각 실행할 수 있다.

```bash
source /opt/ros/humble/setup.bash
source /home/dz/my_autonomous_vehicle/install/setup.bash

ros2 run lio_sam ouster_range_to_cloud.py --ros-args \
  -p use_sim_time:=true \
  -p metadata_bag:=/home/dz/datasets/malaga_os1/metadata_rosbag/metadata_rosbag
```

다른 터미널에서:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/dz/datasets/malaga_os1/etsii_rosbag/etsii_rosbag --clock --rate 0.5
```

터미널 2에서 NDT를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/dz/my_autonomous_vehicle/install/setup.bash

ros2 launch lio_sam ndt_localization.launch.py \
  map_path:=/home/dz/maps/malaga/GlobalMap.pcd
```

RViz2에서 Fixed Frame을 `map`으로 설정하고 PointCloud2(`/ndt/map`,
`/ndt/aligned_points`), Path(`/ndt/path`)를 추가한다. **2D Pose Estimate**로 시작 위치와 yaw를
지정한다.

## 13. TF와 동시 실행 주의점

Mapping에서는 LIO-SAM이 odometry 계열 TF를 만들고 launch가 정적 `map → odom`을 발행한다.
NDT에서는 직접 `map → os_lidar`를 발행한다. 동일 child frame에 서로 다른 경로의 TF를 동시에
발행하면 RViz에서 frame이 튀거나 TF tree가 모순될 수 있다.

권장 운용은 다음 두 mode를 분리하는 것이다.

```text
Mapping mode:
  range converter + LIO-SAM + RViz + bag
  결과: GlobalMap.pcd 생성

Localization mode:
  range converter + NDT + RViz + bag/실센서
  입력: GlobalMap.pcd
  결과: 현재 map pose
```

## 14. 현재 프로젝트 상태

2026-07-22 확인 기준:

- ROS 2 배포판: Humble
- `lio_sam`과 `my_vehicle_description` package가 workspace에 존재한다.
- `lio_sam_ndtLocalization` 실행 파일이 build/install되어 있다.
- ETSII bag과 metadata bag이 모두 존재한다.
- LIO-SAM mapping과 지도 저장을 완료했다.
- `/home/dz/maps/malaga/GlobalMap.pcd`가 존재한다(3,057,222점, 약 48.9 MB).
- corner/surface map과 736개 key pose의 trajectory/transform PCD도 함께 저장했다.
- bag 마지막 구간에서 IMU preintegration이 GTSAM underconstrained 예외로 종료됐지만,
  mapOptimization에 누적된 keyframe 지도는 저장 서비스로 정상 보존했다.

## 15. 점검 명령

```bash
# package/executable 확인
ros2 pkg executables lio_sam

# bag 토픽과 기록 시간 확인
ros2 bag info /home/dz/datasets/malaga_os1/etsii_rosbag/etsii_rosbag

# pipeline 토픽 확인
ros2 topic list | sort

# cloud 발행 속도
ros2 topic hz /ouster/points

# LIO-SAM odometry 확인
ros2 topic echo /lio_sam/mapping/odometry --once

# NDT 결과와 fitness 확인
ros2 topic echo /ndt/odometry --once

# TF tree 생성(tf2_tools가 설치된 경우)
ros2 run tf2_tools view_frames
```

## 16. 추천 학습 순서

1. `launch/malaga_mapping.launch.py`에서 어떤 node가 동시에 실행되는지 본다.
2. `scripts/ouster_range_to_cloud.py`에서 range image가 XYZ/ring/t로 바뀌는 과정을 본다.
3. `imageProjection.cpp`의 subscription과 deskew 흐름을 본다.
4. `featureExtraction.cpp`에서 corner/surface 선택 과정을 본다.
5. `mapOptmization.cpp`에서 keyframe, factor graph, loop closure, save map을 본다.
6. `ndtLocalization.cpp`의 `scanCallback()`을 따라 target/source/guess/result를 본다.
7. 두 YAML에서 파라미터 하나씩 바꾸며 동일 bag 구간으로 결과를 비교한다.

이 순서로 보면 개별 수식보다 먼저 ROS node 사이의 데이터 흐름이 잡혀 전체 코드를 이해하기
쉽다.
