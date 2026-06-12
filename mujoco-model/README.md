# MuJoCo 최소 Actuator 모델

이 폴더는 외부 로봇 프로젝트에서 참고하거나 복사해 쓸 수 있는 최소 MJCF actuator 예제를 담고 있습니다.

## 파일

- `actuator.xml`: hinge joint 1개와 motor actuator 1개로 구성된 단독 실행 가능한 MuJoCo 모델

## 실행

MuJoCo viewer 또는 `simulate`에서 바로 열 수 있습니다.

```bash
simulate mujoco-model/actuator.xml
```

제어 입력 `actuator_motor`의 범위는 `-1`부터 `1`까지입니다. `gear="1"`이므로 입력값은 그대로 joint에 걸리는 토크 스케일로 사용됩니다.

## uv simulator 실행

이 폴더는 `uv` Python 프로젝트이기도 합니다. MuJoCo viewer를 띄우고 actuator에 명령을 넣으려면 아래처럼 실행합니다.

```bash
cd mujoco-model
uv run actuator-sim --mode sine --amplitude 0.5 --frequency 1.0
```

macOS에서는 MuJoCo viewer 요구사항 때문에 실행 스크립트가 자동으로 `mjpython`으로 재실행됩니다.

사용 가능한 구동 모드는 `hold`, `step`, `sine`, `chirp`입니다.

```bash
uv run actuator-sim --mode step --amplitude 0.5 --step-time 1.0
uv run actuator-sim --mode chirp --amplitude 0.5 --duration 20.0 --chirp-start 0.1 --chirp-end 5.0
```

GUI 없이 모델 로드와 구동 루프만 확인하려면 `--no-viewer`를 사용합니다.

```bash
uv run actuator-sim --no-viewer --mode step --duration 2.0
```

시뮬레이터는 `actuator_angle`, `actuator_velocity`, `actuator_torque` 센서 값을 주기적으로 출력합니다.

## SysID 로그로 튜닝할 파라미터

`monitor/`의 chirp 또는 step capture CSV는 아래 컬럼을 저장합니다.

```text
sequence_kind,sequence_target,sequence_value,timestamp_ms,row_kind,command_target,command_value,angle,velocity,torque,temperature,i_q,i_d,debug_kind,debug_value
```

실제 MuJoCo 동역학에 반영되는 최소 튜닝 파라미터는 아래와 같습니다.

| 대상 | XML 위치 | 로그에서 주로 쓰는 값 |
| --- | --- | --- |
| 토크 스케일 | `actuator_motor gear` | `TorqueRef`, `torque`, `i_q` |
| 명령 제한 | `actuator_motor ctrlrange` | `command_value` |
| 출력 토크 제한 | `actuator_motor forcerange` | 포화된 `torque`, `velocity` 응답 |
| 입력 지연 | `actuator_motor delay` | command row와 status row 사이의 응답 지연 |
| 회전자 관성 | `actuator_joint armature` | torque step의 `velocity` 상승 기울기 |
| 링크/부하 관성 | `actuator_link inertial` | torque step의 `velocity` 상승 기울기 |
| 점성 마찰 | `actuator_joint damping` | chirp의 `velocity` 응답 |
| 정지 마찰 | `actuator_joint frictionloss` | 작은 step에서 움직이기 시작하는 `torque` |

`sequence_target`이 `TorqueRef`인 로그는 actuator 자체 식별에 가장 직접적입니다. `SpeedRef`와 `AngleRef` 로그는 firmware의 외부 제어 루프 응답까지 포함하므로, 먼저 `TorqueRef`로 위 파라미터를 맞춘 뒤 제어기 튜닝에 사용하세요.

권장 순서는 `delay`, `gear`, 관성(`armature`, `inertial`), 마찰(`damping`, `frictionloss`), 제한(`ctrlrange`, `forcerange`)입니다. 입력을 전류[A]로 넣을 경우 `gear`를 토크 상수[Nm/A]로 두고 `ctrlrange`를 전류 제한으로 맞춥니다. 입력을 토크[Nm]로 넣을 경우 `gear="1"`을 유지하고 `ctrlrange`와 `forcerange`를 토크 제한으로 맞춥니다.

`sensor`의 `actuator_angle`, `actuator_velocity`, `actuator_torque`는 monitor 로그의 `angle`, `velocity`, `torque`와 비교하기 위한 출력입니다.

## 외부 로봇 모델에 가져다 쓰기

다른 MJCF에 붙일 때는 보통 아래 부분만 가져가면 됩니다.

1. `<default class="actuator_hinge">` 블록
2. 구동되는 body의 `<inertial>` 값
3. `<actuator>` 안의 `<motor>` 정의
4. 필요한 경우 `<sensor>` 안의 비교용 센서 정의

예시:

```xml
<actuator>
  <motor name="shoulder_motor" joint="shoulder_joint" ctrllimited="true" ctrlrange="-1 1" forcelimited="true" forcerange="-1 1" gear="1" delay="0"/>
</actuator>
```

로봇의 실제 응답에 맞게 `gear`, `delay`, `ctrlrange`, `forcerange`, joint의 `damping`, `armature`, `frictionloss`, body의 `inertial` 값을 조정하세요.

## 구성 의도

- 외부 mesh나 texture 없이 동작
- actuator 동작 확인에 필요한 body, joint, geom만 포함
- 복사해서 다른 로봇 모델에 붙이기 쉬운 class 기반 정의 사용
