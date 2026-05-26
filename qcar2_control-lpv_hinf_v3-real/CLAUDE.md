# qcar2_control_2 패키지 규칙

## ⛔ 빌드 및 ROS 명령 금지 (현 경로)

**현 워크스페이스 경로 `/home/nvidia/Documents/ACC_Development/Development/ros2`에서는
빌드 및 ROS 명령을 절대 실행하지 않는다.**

금지되는 명령 예시:
- `colcon build`, `colcon test`
- `ros2 launch`, `ros2 run`, `ros2 node`, `ros2 topic`, `ros2 param` 등 모든 `ros2 ...`
- `rosdep install`, `source install/setup.bash` 후 ROS 동작
- `build/`, `install/`, `log/` 디렉토리에 대한 쓰기/삭제

이유:
- 빌드 및 실행은 별도 환경(예: docker 컨테이너 `/workspaces/isaac_ros-dev/ros2/`)에서 수행됨
- 현 경로의 `build/`, `install/`은 다른 워크스페이스를 가리키는 sym-link / 메타데이터일 수 있어
  손대면 다른 환경의 빌드 상태가 깨질 수 있음
- 실차 환경에서 잘못된 ROS 노드 실행은 안전 사고로 이어질 수 있음

**허용되는 작업:**
- 소스 파일 읽기/수정 (`src/`, `launch/`, `config/`, `data/`)
- `git` 명령 (커밋, 브랜치, cherry-pick 등)
- 정적 분석, 문법 검사 (컴파일/링크 없이)

**빌드 결과 검증이 필요하면**: 사용자에게 다른 환경에서 빌드해 달라고 요청한다.
본인이 직접 빌드하지 않는다.

## Sim/Real 브랜치 워크플로우

이 패키지는 두 환경에서 동시에 개발됨:
- **QLabs 시뮬레이션 환경** (별도 머신): `lpv_hinf_v3` 등 sim 계열 브랜치
- **실차 QCar2 환경** (현 경로): `*-real` 접미사 브랜치 (예: `lpv_hinf_v3-real`)

두 환경은 같은 `origin`을 공유하지만, 일부 코드/런치/파라미터가 다름.

### 관리 방식: cherry-pick + 커밋 분리

브랜치는 분리하되, 공통 변경은 `git cherry-pick`으로 한쪽에서 다른 쪽으로 옮긴다.
이게 동작하려면 **커밋 단위를 잘 쪼개는 것**이 필수.

### 커밋 분리 규칙

1개 커밋에는 **1개 범주**의 변경만 담는다:

- **공통 변경** (sim/real 모두에 필요): 로직 개선, 버그 수정, 새 알고리즘 등
- **sim 전용 변경**: QLabs 토픽/파라미터, 시뮬레이터 가정에 의존하는 코드
- **real 전용 변경**: 실차 하드웨어 인터페이스, 안전 제한, 실측 보정 등

커밋 메시지 접두사 권장:
- `Common:` — 양쪽 환경에 적용될 변경
- `Sim:` — 시뮬레이션 전용
- `Real:` — 실차 전용

### cherry-pick 절차

sim 브랜치(`lpv_hinf_v3`)에서 작업한 공통 변경을 real 브랜치로 옮길 때:

```bash
# 1) sim 쪽에서 공통 변경을 별도 커밋으로 만든다
git checkout lpv_hinf_v3
# ... 코드 수정 ...
git commit -m "Common: <변경 내용>"     # 예: 해시 abc1234
git push

# 2) 실차 머신에서 pull 후 cherry-pick
git checkout lpv_hinf_v3-real
git fetch
git cherry-pick abc1234
git push
```

여러 커밋을 한 번에:
```bash
git cherry-pick abc1234 def5678
# 또는 범위 (앞쪽 커밋은 제외, 뒤쪽은 포함)
git cherry-pick abc1234..ghi9012
```

### 충돌이 났을 때

cherry-pick 도중 충돌이 나면:
```bash
# 충돌 파일 수정 후
git add <충돌 파일>
git cherry-pick --continue

# 또는 중단
git cherry-pick --abort
```

충돌이 자주 난다면 커밋 분리가 부족하다는 신호 → 다음 커밋부터 더 잘게 쪼갠다.

### 환경 차이가 큰 부분은 파라미터/런치로 흡수

코드 자체가 sim/real에서 동일하지만 값만 다른 경우(예: PID 게인, 토픽 이름)는
브랜치 분기 대신 **launch 파일 / config yaml**로 빼서 양쪽에서 같은 소스를 쓰게 한다.
그래야 cherry-pick 부담이 줄어든다.

### 점검 명령

```bash
# 원격과의 차이 확인 (working tree가 아닌 커밋 비교)
git log HEAD..@{u} --oneline
git diff HEAD..@{u}

# 다른 브랜치에 있는데 현 브랜치엔 없는 커밋
git log lpv_hinf_v3-real ^lpv_hinf_v3 --oneline
```
