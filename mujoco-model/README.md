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

## 외부 로봇 모델에 가져다 쓰기

다른 MJCF에 붙일 때는 보통 아래 두 부분만 가져가면 됩니다.

1. `<default class="actuator_hinge">` 블록
2. `<actuator>` 안의 `<motor>` 정의

예시:

```xml
<actuator>
  <motor name="shoulder_motor" joint="shoulder_joint" ctrllimited="true" ctrlrange="-1 1" gear="1"/>
</actuator>
```

로봇의 실제 토크 스케일에 맞게 `gear`, `ctrlrange`, joint의 `damping`, `armature`, `frictionloss` 값을 조정하세요.

## 구성 의도

- 외부 mesh나 texture 없이 동작
- actuator 동작 확인에 필요한 body, joint, geom만 포함
- 복사해서 다른 로봇 모델에 붙이기 쉬운 class 기반 정의 사용
