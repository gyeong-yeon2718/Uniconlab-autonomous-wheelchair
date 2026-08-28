# 조기 회피 + 지오메트릭 인지 롤백

기준 커밋: `db6b814` (main) + `origin/fix/static-person-trajectory-bypass` 머지.
NUC에서 어젯밤 돌던 person-bypass 노드들은 main에 머지돼 있지 않아, 실제로 돌던
그래프와 맞추려면 이 머지가 먼저 필요합니다.

근거 데이터는 전부 2026-08-27 23:53 ~ 08-28 00:20 주행입니다
(`localization_trials/blackbox_20260827_235548.bag`,
`localization_trials/person_bypass_20260827_235916.bag`).

## 1. 정지한 사람도 정지한 사물과 같은 시점에 경로를 만들기 시작한다

`cluster_guard.avoidance_decision`에 `APPROACH`가 생겼습니다.

- 정지한 **사물**: 예전부터 `PLAN_AHEAD_M = 8 m`에서 `GO_ROUND`.
- 정지한 **사람**: 통과 권한이 확정되기 전까지 `WAIT`(정지)였고,
  10초 확인 창을 **선 채로** 소진한 뒤 멈춘 자리에서 권한이 났습니다.

그날 permit이 활성화된 6개 트랙의 최초 인지 거리와 권한 발생 거리:

| track | 최초 인지 | 권한 | 결과 |
|---|---|---|---|
| 286 | 7.51 m | 2.00 m | 게이트 거부 |
| 439 | 5.28 m | 2.03 m | 거부 → 급정지 → 41°/s 피벗 |
| 753 | 2.99 m | 1.90 m | 10.1초 교착 |
| 787 | 4.88 m | 2.11 m | 거부 |
| 1696 | 7.90 m | **5.46 m** | **게이트 차단 0회, 통과** |

확인 시간(`PERSON_BYPASS_CONFIRM_S = 10.0`)과 통과 권한 조건은 **그대로**입니다.
바뀐 것은 그 시간을 서서 보내느냐 접근하며 보내느냐 뿐입니다. `APPROACH` 동안:

- 사람이 플래너 geometry에 들어가고, bypass 여유(`PERSON_BYPASS_CLEARANCE_M`)와
  bypass 속도(`0.35 m/s`)가 적용됩니다.
- 통과 권한은 없습니다. 정지 반경 안으로 들어가면 여전히 `WAIT` → 정지입니다.
- 움직이는 사람에게는 절대 나오지 않습니다. 걸어오는 사람의 geometry를 롤아웃
  스코어러에 넘기는 것은 `test_dwa_policy`가 존재하는 이유인 결함이고, 그대로
  막혀 있습니다.
- pursuit/MPC 프로파일은 `APPROACH`를 실행할 수 없어 예전 `WAIT` 자리에서 섭니다.

상태가 `/waypoint_follower/status`에 `DWA:APPROACH`로 나옵니다. 그날 로그에서
접근 구간이 평범한 추종과 구분되지 않았던 문제도 이걸로 해결됩니다.

## 2. 플래너가 게이트와 같은 모양을 클리어한다

`safety_gate`가 거부하는 것은 **회전하는 사각형**입니다
(앞뒤 0.50 + 여유 0.15, 반폭 0.30 + 여유 0.15 → 코너 `hypot(0.65,0.45)=0.79 m`).
플래너는 롤아웃 중심 기준 **반경 0.50 m 원판**을 봤습니다. 주석에는
"safety_gate의 veto 기하에 맞췄다"고 적혀 있었지만 맞춘 것은 반폭 하나뿐입니다.

대가: `REQUESTED_PATH_COLLISION` 139회, 그중 130회가 00:11:44부터 10초 동안.
그 10초 내내 플래너는 `+0.500 rad/s`(최대 요레이트)를 요청했고, 시맨틱 계층은
막지 않았으며, 계획 0.325 m/s에 실제 평균 0.023 m/s였습니다.

이제 두 쪽 다 `motion_safety`의 같은 상수와 같은 사각형을 씁니다.

- `OBSTACLE_FLOOR_M`은 이제 **사각형 바깥 여유**입니다. 값은 유도됩니다:
  `0.50 - (0.30 + 0.15) = 0.05`. 옆면 여유는 예전 원판과 **동일**하고,
  새로 거부되는 것은 원판이 표현할 수 없던 것 — 정면 0.70 m(0.50이 아니라),
  그리고 선회 중 0.79 m까지 뻗는 코너입니다.
- `PERSON_BYPASS_CLEARANCE_M`도 같은 기준으로 `0.80 - 0.45 = 0.35`.
  물리적 여유는 그대로 0.80 m입니다.

성능: 정확한 (후보 × 스텝 × 포인트) 테스트는 20,000 리턴에서 1454 ms로
100 ms 제어 주기를 넘깁니다(`test_obstacle_preview`가 잡습니다). 그래서
`footprint_clearance`는 KD-tree가 이미 주는 중심 거리로 상·하한을 만들고,
그 한계가 판정을 못 내리는 후보에만 정확한 테스트를 돌립니다. 보고되는 값은
항상 하한이라 여유를 과소평가할 수는 있어도 과대평가하지 않습니다.

**RTX 백엔드 버그 동반 수정.** `gpu_dwa_backend`는 `plan()`을 통째로 복제해 두고
main이 2026-08-27에 추가한 `obstacle_floor_m` 인자를 받지 않습니다. 현장 기본값이
`PREFER_DWA_GPU=true`라서, `db6b814` 상태로는 DWA 사이클마다 `TypeError`가 납니다.
인자를 받게 하고, 기하도 CPU와 같은 함수를 쓰게 했습니다.

## 3. 확인 창 동안에도 플래너가 장애물을 받는다

`obstacles`는 `GO_ROUND`, `PERSON_BYPASS`, 그리고 새 `APPROACH`에서 전달됩니다.
예전에는 `GO_ROUND`에서만 전달돼, 확인 창 내내 플래너가 빈 장애물 목록으로
직진 호를 점수 매기고 있었습니다. 필요한 우회 경로는 권한이 떨어지는 순간까지
아예 존재하지 않았습니다.

움직이는 것에는 여전히 전달하지 않습니다.

## 4. 롤아웃 길이는 건드리지 않았습니다

`SIM_DISTANCE_M = 1.05 m`를 늘리자는 제안이 있었지만, `db6b814`에는 이미
`OBSTACLE_PREVIEW_M = 3.0`이 있어 **장애물 거부 지평선은 3 m**입니다. 짧은 것은
조향 호뿐이고, 늘리면 굽은 구간에서 후보 수가 102→78개로 떨어진다는 실측이
`dwa_core` 주석에 있습니다. 개시 거리 문제는 1번과 3번이 해결합니다.

## 5. 장애물 감지 롤백 (`PERCEPTION_PROFILE`)

같은 주행 안에서 00:04:02에 인지 생산자가 교체됐고, 같은 경로 같은 센서로:

```
subtraction ON  (obstacle_clusters)      평균 1.73 개/프레임, p99 5,  최대 7
subtraction OFF (hybrid_geometric)       평균 5.05 개/프레임, p99 16, 최대 23
```

교체 후 프레임의 15.9 %가 8개 이상이었고, 교체 전에는 0 %였습니다.
**학습 검출기 탓이 아닙니다**: 교체 후 발행된 49,818개 객체 중 49,666개가
geometric이고 152개만 PointPillars입니다.

원인 두 가지가 같이 들어와 있었습니다.

1. `hybrid_geometric_objects`가 `FixedMapFilter`를 `KeepAllGeometry`로 갈아끼워
   **고정 지도 차감을 끕니다**(커밋 `3d1f7b6`). 지도에 이미 있는 벽·연석·기둥이
   매 스캔 새 객체로 올라옵니다. "장애물이 없는데 생긴다"가 이것입니다.
2. 런처 기본값이 클러스터링 임계값을 반으로 낮춰 놨습니다
   (`1 / 5 / 80` vs `obstacle_clusters`의 `2 / 8 / 40`).
   하나의 물체가 여러 개로 쪼개집니다.

`PERCEPTION_PROFILE=legacy_geometric`(기본값)이 둘 다 되돌립니다.

**중요 — 왜 `obstacle_clusters.py`로 그냥 되돌리면 안 되는가.**
그 노드는 `frame: "lidar"`로 발행하는데, 지금 소비자들은 `chair_centre`를
요구합니다. 그날 00:04:02 이전 구간이 정확히 그 상태였고,
`lidar` 프레임 요약 5,714건이 **전부** `PERCEPTION_UNUSABLE`이 되어 시맨틱
계층이 100 % 차단했습니다. 그래서 롤백은 hybrid 그래프 **안에서** 합니다:
클러스터링과 지도 차감은 예전 것, 프레임과 스키마(`source`, `chair_centre`)는
지금 것. `REQUIRE_LEARNED=false`면 fusion 노드는 geometric 단독으로 동작합니다.

되돌린 것이지 지운 것이 아닙니다. `PERCEPTION_PROFILE=hybrid_experimental`로
2026-08-27 그래프를 그대로 부를 수 있고, 지도 차감을 끈 이유(지도에 있는 벽에
붙어 선 사람이 벽과 함께 차감될 수 있다)도 여전히 유효합니다.

## 현장에서 돌리는 법

```bash
PERCEPTION_PROFILE=legacy_geometric bash tools/hybrid.sh start
```

확인할 것:

- `hybrid geometry: fixed-map subtraction is ON` 로그 (OFF 경고가 아니라)
- `rostopic echo /perception/objects_summary` 의 객체 수가 한 자리인지
- `/waypoint_follower/status` 에 `DWA:APPROACH`가 8 m 근처에서 나타나는지
- `/safety_gate/status` 의 `trajectory_override_reason` 에
  `REQUESTED_PATH_COLLISION` 이 줄었는지
