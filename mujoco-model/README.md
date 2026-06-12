# MuJoCo 최소 Actuator 모델

이 폴더는 외부 로봇 프로젝트에서 참고하거나 복사해 쓸 수 있는 최소 MJCF actuator 예제를 담고 있습니다.

## 파일

- `actuator.xml`: 다른 MJCF에서 `<include>`로 재사용할 `gm3506` actuator default class
- `actuator_standalone.xml`: `actuator.xml`의 class로 테스트용 단일 축을 만든 MuJoCo 모델

단독 테스트 모델에서는 높이 60 mm housing을 고정하고, 회전 확인용 `visual_arm`만 `actuator_joint`에 붙어 움직입니다. `visual_arm`은 질량 0 표시용 geom이라 동역학에는 영향을 주지 않습니다. 회전 출력부의 동역학은 테스트 모델의 `actuator_output` 관성값으로 표현합니다.

## 실행

MuJoCo viewer 또는 `simulate`에서 standalone 모델을 바로 열 수 있습니다.

```bash
simulate mujoco-model/actuator_standalone.xml
```

제어 입력 `actuator_motor`는 position actuator의 목표 각도이며 범위는 `-3.14`부터 `3.14`까지입니다.

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
| Position gain | `gm3506 position kp` | `AngleRef`, `angle` |
| 명령 제한 | `actuator_motor ctrlrange` | `command_value` |
| 출력 토크 제한 | `gm3506 position forcerange` | 포화된 `torque`, `velocity` 응답 |
| 회전자 관성 | `gm3506 joint armature` | step의 `velocity` 상승 기울기 |
| 출력/부하 관성 | `actuator_output inertial` | step의 `velocity` 상승 기울기 |
| 점성 마찰 | `gm3506 joint damping` | chirp의 `velocity` 응답 |
| 정지 마찰 | `gm3506 joint frictionloss` | 작은 step에서 움직이기 시작하는 `torque` |

권장 순서는 position gain(`kp`, `kv`), 관성(`armature`, `inertial`), 마찰(`damping`, `frictionloss`), 제한(`ctrlrange`, `forcerange`)입니다.

`sensor`의 `actuator_angle`, `actuator_velocity`, `actuator_torque`는 monitor 로그의 `angle`, `velocity`, `torque`와 비교하기 위한 출력입니다.

## 외부 로봇 모델에 가져다 쓰기

다른 MJCF에서는 루트 `<mujoco>` 아래에서 default class를 include하고, 각 로봇 joint/body에서 필요한 class를 선택합니다.

```xml
<mujoco model="robot">
  <compiler angle="radian"/>

  <include file="path/to/actuator.xml"/>

  <worldbody>
    <body name="upper_arm">
      <geom name="elbow_housing" class="gm3506" type="cylinder" fromto="0 -0.03 0 0 0.03 0" size="0.025"/>

      <body name="forearm">
        <joint name="elbow_joint" class="gm3506" type="hinge" axis="0 1 0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="elbow_motor" class="gm3506" joint="elbow_joint" ctrlrange="-1.69 1.69"/>
  </actuator>
</mujoco>
```

여러 actuator를 쓰는 로봇에서는 joint, actuator, sensor 이름을 로봇 모델 쪽에서 각각 다르게 정하고, `gm3506` class만 재사용합니다. 로봇의 실제 응답에 맞게 `kp`, `kv`, `ctrlrange`, `forcerange`, joint의 `damping`, `armature`, `frictionloss`, body의 `inertial` 값을 조정하세요.

## 구성 의도

- 외부 mesh나 texture 없이 동작
- 단독 모델에는 actuator 동작 확인에 필요한 body, joint, geom만 포함
- 로봇 모델에서 이름 충돌 없이 쓰기 쉬운 class 기반 정의 사용
