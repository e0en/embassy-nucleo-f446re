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

## CAN SysID 기반 자동 튜닝

`actuator-sysid`는 CAN으로 실기기 모터를 MotionControl 모드로 구동하며 데이터를 수집하고, 같은 실행에서 MuJoCo actuator 파라미터를 least-squares로 맞춥니다. sysid 명령은 `angle_ref`만 바꾸고 `dq_ref=0`, `tau_ff=0`으로 고정합니다. 실기기에 보낼 MotionControl의 `kp/kd`는 `--motion-kp`, `--motion-kd`로 지정합니다. 펌웨어와 시뮬레이터 모두 `kp`는 출력축 기준 `Nm/rad`, `kd`는 출력축 기준 `Nm/(rad/s)`로 해석합니다. 튜닝에서는 명령 값을 초기값으로 쓰되, 실제 position 응답에서 유효 `kp/kd`를 다시 추정합니다. `--amplitude`는 angle reference amplitude[rad]입니다. 안전을 위해 실제 구동은 `--yes`를 명시해야 시작됩니다.

```bash
cd mujoco-model
uv run actuator-sysid --channel can0 --motor-id 0x0F --amplitude 0.5 --motion-kp 20 --motion-kd 0.5 --duration 30 --yes
```

수집만 하려면:

```bash
uv run actuator-sysid collect --capture sysid_capture.csv --amplitude 0.5 --motion-kp 20 --motion-kd 0.5 --duration 30 --yes
```

이미 수집한 capture를 다시 튜닝하려면:

```bash
uv run actuator-sysid tune --capture sysid_capture.csv
```

multi-start를 여러 코어로 병렬 실행하려면 `--workers`를 지정합니다. `0`은 사용 가능한 CPU와 `--multi-start` 중 작은 값을 자동으로 씁니다.

```bash
uv run actuator-sysid tune --capture sysid_capture.csv --multi-start 16 --workers 0
```

튜닝은 stage fitting으로 진행합니다. 전체 동적 구간에서 `delay_s`를 먼저 맞추고, `small_prbs`/`medium_prbs`/`sweep` 구간에서 유효 Motion gain `kp/kd`를 맞춥니다. 그 다음 `pulses`/`large_prbs` 구간에서 `armature`를 맞추고, `small_prbs`/`medium_prbs`/`sweep` 구간에서 `damping`과 `frictionloss`를 맞춥니다. 각 stage는 multi-start least-squares를 사용합니다.

튜닝 대상은 `joint damping`, `joint frictionloss`, `joint armature`, `delay_s`, `kp`, `kd`입니다. 시뮬레이션 torque는 `tau = kp * (angle_ref - angle) - kd * velocity`로 계산하고 `1.5 Nm`으로 clip합니다. residual은 측정 position과 시뮬레이션 position의 차이만 사용합니다.

최종 출력의 `least-squares cost`는 최적화 내부의 normalized residual 값이라 물리 단위로 해석하기 어렵습니다. 실제 적합 품질은 함께 출력되는 `fit metrics`를 봅니다. `angle_rmse_rad`가 핵심 지표이고, `velocity_rmse_rad_s`, `velocity_corr`는 참고용입니다. 같은 값들은 `actuator_sysid.json`의 `metrics`에도 저장됩니다. 튜닝이 끝나면 phase별 command/measured/simulated angle 비교 plot도 기본적으로 `actuator_sysid_plots.png`에 저장됩니다. 경로를 바꾸려면 `--output-plot`을 지정합니다.

원본 `actuator.xml`에 바로 반영하려면 `--write`를 사용합니다.

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
