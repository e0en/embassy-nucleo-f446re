# stm32-kit

STM32 기반 모터 제어 펌웨어 프로젝트.

## Firmware roles

기본 빌드는 저장된 flash calibration을 사용하는 main firmware다. 저장된 calibration이 없으면 모터 튜닝을 자동 실행하지 않는다.

```bash
cargo build --release
```

튜닝은 `tuner-fw` feature로 별도 빌드한다. tuner firmware는 calibration을 다시 수행하고 flash config page에 저장한 뒤 모터를 끈다.

```bash
cargo build --release --no-default-features --features tuner-fw
```

권장 순서:

1. tuner firmware를 flash해서 calibration을 저장한다.
2. main firmware를 flash해서 저장된 calibration으로 구동한다.

main firmware를 flash할 때 chip erase로 마지막 flash config page를 지우면 저장된 calibration도 사라진다. config page를 보존하는 flash 절차를 사용해야 한다.

## CAN 운영 메모

호스트 `can0` 또는 USB CAN 어댑터 상태가 꼬였을 때의 복구 절차는 [CAN_RECOVERY.md](./CAN_RECOVERY.md)에 정리되어 있다.

진단 정보 수집 스크립트:

```bash
./scripts/can_diag.sh can0
```
