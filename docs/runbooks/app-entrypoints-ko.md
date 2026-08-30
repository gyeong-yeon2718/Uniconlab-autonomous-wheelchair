# 앱 버튼이 어느 배포판을 움직이는가 (운영 런북)

앱의 [로컬 켜기] · [주행 시작] · [주행 정지] · [스택 내리기] 네 버튼은
브릿지(`scripts/ros1_bluetooth_bridge.py`, `JobRunner.JOBS`)가 허용 목록으로
가진 홈 스크립트 네 개를 실행합니다.

    ~/start_wheelchair_localization.sh   [로컬 켜기]
    ~/go.sh                              [주행 시작] / [시동 + 주행]
    ~/stop.sh                            [주행 정지] · E-STOP 스크립트 경로
    ~/stop_stack.sh                      [스택 내리기]

이 네 개는 NUC에서 손으로 관리됐고 git에 없었습니다. 그래서 갈라졌습니다 —
2026-08-28에 [로컬 켜기]는 한 배포판을 올리고 [주행 시작]은 다른 배포판을
몰았는데, 어느 쪽에서도 그게 보이지 않았습니다.

지금은 넷 다 `tools/wheelchair_entry.sh` 한 파일로 들어가는 세 줄짜리
래퍼이고, 그 래퍼들은 설치 스크립트가 **한 번에 네 개 모두** 씁니다. 하나만
고칠 방법이 없습니다.

## 설치 / 전환

배포판을 바꾸는 것은 `current` 심볼릭 링크가 아니라 **설치 행위**입니다.
몰고 싶은 배포판의 트리에서:

```bash
bash ~/wheelchair_deploys/<이름>/source-linux/tools/install_operator_entrypoints.sh
systemctl --user restart wheelchair-bt-bridge
```

원본은 `~/<이름>.pre-wheelchair-entry`로 백업됩니다.
되돌리려면 `... /install_operator_entrypoints.sh --revert`.

## 확인 — 필드 나가기 전 한 줄

```bash
bash ~/wheelchair_deploys/<이름>/source-linux/tools/wheelchair_entry.sh check
```

- 네 래퍼가 **같은** entry 파일을 가리키는지 (`DIVERGED`면 아님)
- `ws/src/static_livox_localization` 심볼릭 링크가 이 배포판 안을 가리키는지

두 번째가 실제로 도는 코드입니다. catkin이 설치하는 릴레이 스텁이 소스 파일을
exec 하므로, 디렉터리 이름도 `REVIEWED_COMMIT`도 아니고 **그 심볼릭 링크**가
무엇이 굴러가는지를 결정합니다. 2026-08-30에 `current`가 가리키던 배포판의
심볼릭 링크는 아예 다른 배포판을 가리키고 있었습니다.

`resolve`는 같은 정보를 출력만 하고 아무것도 실행하지 않습니다. 주행 명령마다
같은 배너가 먼저 찍히므로 `/tmp/bt_job_*.log`에 무엇을 몰았는지 남습니다.

## 왜 `current`를 따라가지 않는가

`wheelchair_entry.sh`는 **자기가 들어있는 배포판**을 씁니다. `current`는 다른
사람이 옮길 수 있는 심볼릭 링크이고, 실제로 2026-08-30 한 세션 안에서 두 번
움직여 다른 브랜치의 배포판으로 끝났습니다. `cat ~/go.sh` 한 번으로 어느
트리가 모는지 보여야 합니다.

일부러 다른 걸 지목할 때만 `WHEELCHAIR_DEPLOY=<이름>`을 씁니다.

## 정지 경로

`drive-stop`과 `estop`은 **`hybrid.sh`를 거치지 않습니다.** `hybrid.sh stop`은
`BASE_STOP:-$HOME/stop.sh`로 풀리는데 `~/stop.sh`가 래퍼이므로, 그 경로는 매
바퀴마다 "아직 일어나지 않은 정지"를 반복하는 루프가 됩니다. 배포판의
`tools/stop.sh`로 곧장 갑니다 — 아무것도 검사하지 않는, 원래 그런 스크립트로.

E-STOP 자체는 브릿지가 `mode_cmd=77`을 직접 퍼블리시합니다. 배포판과 무관하게
같은 동작이고, 셸을 거치지 않으므로 스크립트가 무엇이든 즉시 듣습니다.
`~/stop.sh`도 같은 두 가지(모드 77 + 팔로워 정지)를 같은 순서로 합니다.

## [시동 + 주행]이 이제 프리플라이트를 지납니다

예전에는 `arm_and_drive`가 모드 65를 퍼블리시하고 팔로워 start 서비스를 직접
불렀습니다. `go_hybrid.sh`의 검사 — 노드 8개, CuPy/RTX DWA 백엔드,
PointPillars, `hybrid_preflight.py`, `person_bypass_preflight.py` — 가 하나도
돌지 않는 유일한 경로였고, 하필 그게 **기동 직후 평소에 출발하는 방법**입니다
(베이스가 수동으로 쉬고 있으면 앱의 [주행 시작]도 이 명령을 보냅니다).

지금은 모드 65 → 모터 컨트롤러의 auto 에코 확인 → `[주행 시작]`과 **같은**
`~/go.sh` 순으로 갑니다. 거부될 수 있는 것은 전부 모드 65가 나가기 **전에**
거부합니다.

따라서 프리플라이트가 실패하면 이 버튼도 거부합니다. 그게 의도입니다.
프로필을 껐다면 (`PERCEPTION_PROFILE=legacy_geometric`은
`START_POINTPILLARS=false`) `go_hybrid.sh`는 `tools/perception_profile.sh`를
읽어 같은 기준을 씁니다 — 기동할 때 끈 검출기를 주행할 때 요구하지 않습니다.

## 브릿지 실행 인자

`PROFILE`이 유닛 파일에 없으면 systemd 프로세스에는 셸 프로필이 없어서
[로컬 켜기]가 `pursuit`로 올라옵니다. 현재 유닛은
`Environment=PROFILE=dwa SAFETY_POLICIES=true`.
브릿지는 `--allow-commands --allow-scripts` 둘 다 있어야 버튼이 삽니다.
