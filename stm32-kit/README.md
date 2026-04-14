# stm32-kit

STM32 기반 모터 제어 펌웨어 프로젝트.

## CAN 운영 메모

호스트 `can0` 또는 USB CAN 어댑터 상태가 꼬였을 때의 복구 절차는 [CAN_RECOVERY.md](./CAN_RECOVERY.md)에 정리되어 있다.

진단 정보 수집 스크립트:

```bash
./scripts/can_diag.sh can0
```
