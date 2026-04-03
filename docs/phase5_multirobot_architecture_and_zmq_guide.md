# Pinky Multi-Robot Architecture & ZeroMQ Communication Guide

현재 진행 중인 마이그레이션 프로젝트는 ROS 2(DDS 기반)에 의존하던 구조를 탈피하여, **ZeroMQ(ZMQ)와 Protocol Buffers(Protobuf)** 를 이용한 가볍고 빠른 순수 C++ 아키텍처로 전환하는 과정입니다.

본 문서는 3대의 로봇(ID: 7번 포함)을 동시에 제어하기 위해 적용된 시스템 아키텍처와, 구현된 통신 방식(ZeroMQ)이 과거 ROS 2의 통신 방식(Topic, Service, Action)과 어떻게 대응되는지 정리한 가이드입니다.

---

## 1. 현재 통신 방식 (ZeroMQ + Protobuf) 이해하기

### 1-1. ZeroMQ란?
ZeroMQ(ZMQ)는 단순한 소켓(TCP/UDP) 통신을 넘어, **메시지 지향(Message-oriented)** 통신을 매우 쉽고 빠르며 안정적으로 구현할 수 있게 해주는 고성능 비동기 메시징 라이브러리입니다. 브로커(중앙 서버)가 필요 없는 P2P 방식이 기본이며, ROS 2의 DDS(Data Distribution Service)를 훨씬 가볍게 대체할 수 있습니다.

### 1-2. Protocol Buffers (Protobuf) 란?
구글이 개발한 데이터 직렬화 포맷입니다. ZMQ는 "바이트 배열"만 전송하므로, 이 바이트 배열 안에 어떤 데이터(속도, 배터리, 이미지 등)가 들어있는지 정의하고 압축/해제하는 역할이 필요합니다. 이를 위해 `pinky.proto` 파일을 정의하여 C++과 Python 양쪽에서 동일한 구조로 데이터를 주고받습니다.

### 1-3. 현재 구현된 프로토콜 (포트 및 패턴)
각 로봇은 고유의 IP에서 2개의 포트와 ZMQ 통신 패턴을 조합하여 사용합니다.

1. **포트 9200 (PUB / SUB 패턴) - 텔레메트리 및 데이터 스트리밍**
   *   **로봇(PUB)**: 센서 데이터(오도메트리, IMU, 배터리)와 카메라 프레임(`VideoStream`)을 생성하여 9200 포트로 일방적으로 흩뿌립니다(Publish).
   *   **관제탑(SUB)**: 로봇의 IP:9200에 접속하여 필요한 토픽(`T` 텔레메트리, `V` 비디오)만 골라서 수신합니다(Subscribe).
   *   *(ROS 2의 `Publisher / Subscriber`와 완벽히 동일합니다.)*

2. **포트 9100 (REQ / REP 패턴) - 제어 명령 및 상태 확인**
   *   **관제탑(REQ)**: 로봇을 움직이기 위한 명령(`CmdVel`, `NavGoal`)을 특정 로봇의 9100 포트로 보냅니다(Request).
   *   **로봇(REP)**: 명령을 수신하고, 정상적으로 처리되었는지 응답(`CommandAck`)을 반환합니다(Reply).
   *   *(ROS 2의 `Service`와 유사하며, 제어의 신뢰성을 보장합니다.)*

---

## 2. ROS 2 개념과 ZeroMQ 대응 (Mapping)

과거 ROS 2에서 사용하던 주요 통신 방식들은 현재 ZMQ 아키텍처에서 다음과 같이 구현/대체됩니다.

| ROS 2 개념 | ZeroMQ + Protobuf 대응 방식 | 설명 |
| :--- | :--- | :--- |
| **Topic (토픽)** | **PUB / SUB** (Port 9200) | `SensorTelemetry` 구조체 안에 `odom`, `battery` 등을 넣어 PUB 전송. 수신자는 SUB 소켓으로 구독. |
| **Service (서비스)** | **REQ / REP** (Port 9100) | 관제탑이 `ControlCommand`를 REQ로 전송하면, 로봇이 즉각 `CommandAck`를 REP로 반환. |
| **Action (액션)** | **REQ/REP + 상태 폴링(PUB/SUB)** | 목표(Goal) 설정은 REQ/REP로 전송하고, 주행 중 진행률(Feedback) 및 결과(Result)는 PUB/SUB의 `RobotStatus` 토픽을 통해 관제탑이 지속적으로 확인. |
| **Parameter (파라미터)** | **설정 파일 (`.yaml`) + REQ/REP** | 로봇 구동 시 `robot_config.yaml`을 읽어 초기화하며, 실행 중 변경이 필요한 값은 REQ/REP 명령을 새로 정의하여 처리. |

---

## 3. 다중 로봇 (Multi-Robot) 확장 적용 현황 (구현 완료)

현재 시스템은 1:N 구조(관제탑 1대 - 로봇 최대 3대)를 지원하도록 완벽히 개편되었습니다. (ex: ID 7번 로봇 포함)

### 3-1. 통신 및 식별자 (Robot ID)
*   **로봇 코어 (`pinky_core`)**: `robot_config.yaml` 파일에 `robot: id: "7"`과 같이 로봇의 고유 식별자가 추가되었습니다. 로봇이 송신하는 모든 데이터(오도메트리, 라이다, 배터리, 비디오 프레임 등) 내부 프로토콜(`robot_id` 필드)에 이 ID가 자동으로 주입됩니다.
*   **관제탑 데이터 수신 (`pinky_station`)**: 관제탑은 다중 로봇의 IP들로 각각의 수신 스레드(`ZmqReceiverThread`)를 동시 생성하여 연결합니다. 수신되는 데이터는 모두 `(robot_id, payload)` 형태로 묶여 GUI 이벤트로 전달되므로 데이터가 섞이지 않습니다.

### 3-2. 관제탑 GUI 개편 (멀티 채널 기반)
1.  **로봇 동적 추가 시스템 (Add Robot)**:
    *   툴바에서 단일 IP 입력 구조를 제거하고, `Robot ID`와 `IP`를 입력받아 **[Add Robot]** 하는 방식으로 변경했습니다. 이를 통해 로봇을 런타임에 여러 대 등록할 수 있습니다.
2.  **활성 로봇 제어 (Active Robot Selection)**:
    *   툴바에 **Active Robot** 드롭다운 콤보박스가 추가되었습니다.
    *   이 콤보박스에서 선택된 특정 로봇(예: "7")의 텔레메트리 데이터만 좌측 배터리 상태, 카메라 뷰, 터미널 로그 화면에 필터링되어 출력됩니다.
    *   텔레옵(조이스틱)으로 조종할 때도 "Active Robot"으로 설정된 로봇에게만 제어 명령이 전송됩니다.
3.  **다중 로봇 맵 네비게이션 (`MapWidget`)**:
    *   ZMQ로 수신되는 모든 로봇의 위치(Odometry)를 맵 위에 동시에 마킹합니다.
    *   현재 **Active Robot**은 맵상에서 궤적(Trail)과 함께 파란색(Cyan)으로 강조되어 표시되며, 나머지 대기 중인 로봇들은 회색(Gray) 원으로 위치만 표시됩니다.
    *   맵에서 목표지점을 마우스 클릭(`NavGoal`)하면, 해당 목표 좌표는 "현재 선택된 Active Robot"을 향해 전송됩니다.

---

## 4. 다중 로봇 시스템 사용 가이드

1.  **로봇(단말) 설정**:
    *   각 로봇의 `pinky_core/config/robot_config.yaml`을 열고 `robot:` 섹션 하위의 `id:` 값을 서로 다르게 설정합니다. (예: 로봇A는 `"7"`, 로봇B는 `"8"`)
    *   각 로봇에서 `./pinky_robot` 바이너리를 실행하여 대기 상태로 만듭니다.
2.  **관제탑 실행**:
    *   `uv run python3 -m pinky_station.main` 으로 GUI를 실행합니다.
3.  **로봇 연결**:
    *   툴바의 "Robot ID"와 "IP" 입력 칸에 첫 번째 로봇 정보(예: ID: 7, IP: 192.168.0.40)를 입력하고 **[Add Robot]** 버튼을 누릅니다.
    *   동일한 방식으로 두 번째, 세 번째 로봇 정보도 추가합니다.
4.  **관제 및 제어**:
    *   **[Active Robot]** 드롭다운 박스에서 현재 제어할 로봇을 선택합니다.
    *   카메라 영상, 배터리 정보가 선택된 로봇의 것으로 갱신되는지 확인합니다.
    *   **[Load Map]** 버튼을 눌러 정적 맵을 불러옵니다.
    *   맵 위에 나타난 로봇들의 위치를 확인하고, 맵의 빈 공간을 좌클릭하여 선택된 로봇이 이동할 목표지점(Goal)을 명령합니다.
