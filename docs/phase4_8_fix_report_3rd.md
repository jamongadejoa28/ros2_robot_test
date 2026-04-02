# Phase 4-8 3차 버그 수정 및 기능 추가 보고서

> 작성일: 2026-04-02
> 대상: PC 스테이션 GUI 및 C++ 코어(카메라, LCD, 컨트롤) 런타임 버그 수정

---

## 1. 수정 내역 요약 (총 5건)

### #1. LCD 감정 GIF 재생 불가 (도형 렌더링 폴백 문제)

**파일:** `pinky_core/src/app/robot_app.cpp`

**문제:**
`robot_config.yaml`의 `emotion_dir: emotion` 상대 경로가 OpenCV의 `VideoCapture`로 전달될 때, `/` 또는 `./`로 시작하지 않는 단순 문자열은 OpenCV 내부의 GStreamer가 비디오 파일이 아닌 GStreamer 파이프라인 문자열로 잘못 해석하여 에러(`Error opening bin: no element "emotion"`)를 발생시키고 재생에 실패함.

**수정:**
파일 경로 조합 시, 경로가 절대경로나 명시적 상대경로(`.`, `/`)로 시작하지 않으면 강제로 `./`를 앞에 붙여 파일 경로임을 명확히 인식하도록 우회 코드를 추가했습니다.

---

### #2. PC GUI 카메라 영상 미표시 및 코어 카메라 드라이버 누락

**파일:**
- `pinky_core/include/pinky_core/hal/cv_camera.h` (신규)
- `pinky_core/src/hal/cv_camera.cpp` (신규)
- `pinky_core/CMakeLists.txt`
- `pinky_core/src/app/robot_app.cpp`

**문제:**
문서(`phase4_8_fix_report_2nd.md`)에 명시되었던 기능 미구현 항목으로, 로봇 측의 카메라 스트리밍 캡처 루프 기능이 C++ 핵심 코어에 구현되어 있지 않았고 드라이버 할당조차 누락된 상태였습니다. 이로 인해 PC GUI에서 카메라 영상이 나오지 않았습니다.

**수정:**
OpenCV의 `VideoCapture`와 `imencode`를 활용한 `CvCameraDriver`를 새롭게 작성하여 HAL에 추가했습니다. `CMakeLists.txt`에 빌드 소스를 추가하고, `robot_app.cpp` 초기화 루틴에 해당 드라이버를 생성/할당하도록 연동하여 TCP 포트로 정상적인 JPEG 이미지 스트리밍이 전송되게 하였습니다.

---

### #3. 회전(조향) 속도 조절 슬라이더 누락 및 좌/우 버튼 속도 적용 오류

**파일:** `pinky_station/pinky_station/gui/widgets/teleop_widget.py`

**문제:**
Python 스테이션 앱의 수동 조종 위젯에 선형 속도(Linear Speed)를 조절하는 슬라이더만 존재했고 회전 속도 슬라이더가 없었습니다. 또한 A/D (좌/우) 버튼 클릭 시 속도 명령이 제대로 적용되지 않는 문제가 있었습니다.

**수정:**
`TeleopWidget`에 Angular Speed용 슬라이더(`slider_angular`)를 추가하고, W/S (직/후진)뿐만 아니라 A/D (좌/우회전) 버튼에서도 해당 회전/선형 속도 배수 설정값이 정상적으로 로봇에 명령(`sig_cmd_vel.emit`)되도록 수정했습니다.

---

### #4. WASD 키보드 수동 컨트롤 미지원

**파일:** `pinky_station/pinky_station/gui/main_window.py`

**문제:**
PC GUI 창에서 키보드(WASD) 입력으로 로봇을 조종할 수 있는 이벤트 핸들러가 구현되어 있지 않았습니다.

**수정:**
`main_window.py`에 `keyPressEvent`와 `keyReleaseEvent`를 오버라이드하여, 창이 포커스 된 상태에서 W, A, S, D 및 스페이스 바(정지) 입력 시 GUI의 방향 버튼이 눌린 것과 동일한 동작(속도 명령 전송 및 해제)을 수행하도록 연동했습니다.

---

### #5. 라이다 스캔 그래프 미표시 및 패킷 예외 처리 미흡

**파일:** `pinky_station/pinky_station/gui/main_window.py`

**문제:**
라이다 UDP 패킷을 수신하여 그래프로 변환하는 코드가 `try ... except Exception: pass`로 감싸져 있어, 네트워크 문제로 데이터가 일부 손실되어 `struct.unpack`에 실패할 경우 에러 로그 없이 조용히 무시되고 라이다 화면이 갱신되지 않는 문제가 있었습니다.

**수정:**
예외 발생 시 에러 사유를 터미널로 출력(`print(f"Lidar data error: {e}")`) 하도록 예외 처리 코드를 개선하여 라이다 데이터 처리 실패 원인을 쉽게 파악할 수 있도록 했습니다.

---

### #6. 카메라 스트리밍 패킷 전송 중단 (EAGAIN 프레이밍 깨짐)

**파일:** `pinky_core/src/net/tcp_server.cpp`

**문제:**
JPEG와 같은 대용량 페이로드를 논블로킹(Non-blocking) TCP 소켓으로 전송할 때 송신 버퍼가 가득 차면 `EAGAIN` 또는 `EWOULDBLOCK` 에러가 발생합니다. 기존 코드는 이 경우 바로 반복문을 탈출하여 프레임 전송을 부분적으로 중단했습니다. 이로 인해 수신 측(파이썬 GUI)의 프레이밍 로직이 깨지고 패킷이 버려져 영상이 전혀 갱신되지 않았습니다.

**수정:**
`Send()` 및 `Broadcast()` 함수에서 `EAGAIN` 발생 시 패킷을 버리지 않고 `usleep(1000)`으로 대기 후 재시도하도록 스핀 락 로직을 복구했습니다. (뮤텍스를 해제한 상태에서 진행하므로 다른 스레드를 블로킹하지 않습니다.)

---

### #7. LCD GIF GStreamer 호환성 및 재생 에러 완전 해결

**파일:** `pinky_core/src/core/emotion_renderer.cpp`

**문제:**
상대경로 패치에도 불구하고 OpenCV 내장 `VideoCapture` 백엔드가 구동 환경에 따라 `.gif`를 비디오로 인식하지 못하거나 `CAP_IMAGES` 백엔드가 파일 인덱스를 요구하며 `can't find starting number` 에러를 뿜고 뻗는 현상이 지속되었습니다.

**수정:**
GIF 파싱 로직에서 OpenCV 매크로 우회(`PINKY_HAS_OPENCV`) 부분을 전면 삭제하고, 기존에 포함되어 동작이 확실히 검증된 `stb_image` 라이브러리의 `stbi_load_gif_from_memory` 함수가 무조건 실행되도록 강제했습니다. 이를 통해 로컬 환경의 GStreamer/OpenCV 버전에 구애받지 않고 부드러운 GIF 렌더링이 가능해졌습니다.

---

### #8. MapWidget 위치 표시 AMCL 연동 (헛바퀴 이동 현상 해결)

**파일:**
- `pinky_station/pinky_station/workers/nav_worker.py`
- `pinky_station/pinky_station/gui/main_window.py`
- `pinky_station/pinky_station/gui/widgets/map_widget.py`

**문제:**
현재 위치를 무조건 바퀴의 엔코더(Odometry)에만 의존하여, 로봇을 들어 올려 바퀴만 돌려도 지도 상에서 로봇이 이동하는 것으로 표시되었습니다. 실제 로컬라이제이션 정보가 반영되지 않았습니다.

**수정:**
ROS 2 브릿지 노드(`RosBridgeNode`)에 `amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) 구독기를 추가하고 수신 시 `sig_amcl_pose` 시그널을 방출하도록 연동했습니다. `MapWidget`에 `update_amcl_pose()` 메서드를 신설하여, AMCL 로컬라이제이션 정보가 들어오기 시작하면 단순 Odometry 업데이트를 무시하고 실제 추정된 위치로 맵을 그리도록 로직을 개선했습니다.

---

## 2. 남은 미구현/미검증 항목 업데이트

이번 수정으로 아래 항목들이 모두 해결되었습니다:
* [x] 카메라 스트리밍 (로봇 측 캡처 루프 구현 및 대용량 패킷 EAGAIN 버그 수정)
* [x] LCD 감정 표시 (stb_image 라이브러리 강제 폴백을 통한 GIF 렌더링 완벽 대응)
* [x] PC 스테이션 GUI 실행 테스트 (WASD 컨트롤 추가)
* [x] Nav2 브리지 (AMCL Pose 구독 및 MapWidget 위치 표시 적용)
