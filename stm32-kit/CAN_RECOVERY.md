# CAN Recovery

이 문서는 호스트 `can0` 또는 USB CAN 어댑터가 비정상 상태일 때 재발 대응 절차를 정리한다.

## 빠른 진단

재부팅 전에 먼저 상태를 저장한다.

```bash
./scripts/can_diag.sh can0
```

보드 로그와 함께 다음 항목을 같이 기록한다.

- `CAN: ... TEC=... REC=... last_err=...`
- 호스트 `ip -details link show can0`
- 호스트 `dmesg` 최근 로그

에러 해석은 대략 다음과 같다.

- `TEC` 증가 + `last_err=ack`: 보드는 송신했지만 버스에서 ACK가 돌아오지 않음
- `REC` 증가 + `stuff/form/crc`: 수신 파형 또는 비트타이밍이 깨져 있음
- `BusOff`: 호스트나 보드 중 한쪽이 링크 레벨에서 크게 망가진 상태

## 기본 복구 순서

1. 인터페이스 상태 확인

```bash
ip -details link show can0
```

2. 인터페이스 down/up

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

3. `candump`로 수신 확인

```bash
candump can0
```

4. 어댑터를 뺐다 다시 꽂고 같은 절차 반복

이 단계에서 회복되지 않으면 `SocketCAN`만이 아니라 USB 드라이버나 USB 컨트롤러 상태까지 의심한다.

## 강한 복구 절차

1. 관련 프로세스 종료

```bash
pkill -f candump || true
pkill -f cansend || true
pkill -f cargo || true
```

2. 인터페이스 완전 down

```bash
sudo ip link set can0 down
```

3. 커널 로그 확인

```bash
dmesg | tail -n 100
```

4. USB 장치 재인식 확인

```bash
lsusb
```

5. 가능하면 어댑터가 연결된 USB 버스를 리셋

이 단계는 장치/환경마다 방법이 다르다. 재플러그로 회복되지 않는데 재부팅으로만 회복된다면 USB 버스 또는 드라이버 내부 상태가 꼬였을 가능성이 높다.

6. 그래도 회복되지 않으면 호스트 재부팅

## 재발 시 체크포인트

- 호스트와 펌웨어의 bitrate가 정말 같은지 확인
- `can0`가 `listen-only`나 `down` 상태가 아닌지 확인
- USB 허브를 쓰고 있다면 허브를 우회해 직접 연결해 보기
- 보드 로그가 `ack`인지 `stuff/form/crc`인지 구분해서 기록
- 가능하면 문제 발생 직후 `./scripts/can_diag.sh can0` 결과를 보관

## 이번 사례의 해석

이번 사례는 다음 흐름이었다.

- 보드에서 `ack` 오류가 관찰됨
- 호스트 `can0` 재기동 및 어댑터 재연결만으로는 회복되지 않음
- 호스트 재부팅 뒤 `500 kbps`, 이후 다시 `1 Mbps`에서도 정상 동작

이 패턴은 펌웨어보다는 호스트 `can0` 또는 USB CAN 어댑터의 드라이버/USB 스택 상태가 꼬였을 가능성을 더 강하게 시사한다.
