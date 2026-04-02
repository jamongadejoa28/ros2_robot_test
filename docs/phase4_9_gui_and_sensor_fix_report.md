# Phase 4.9: GUI Display and Sensor Hardware Error Handling Fix Report

## 1. 개요
본 문서는 Pinky PC GUI(`pinky_station`)에서의 화면 미출력 문제, 조작 편의성 개선, 그리고 로봇 코어(`pinky_core`) 센서 하드웨어(BNO055 IMU)의 연결 실패 시 발생하던 시스템 멈춤 현상에 대한 수정 및 개선 내역을 요약합니다.

## 2. 주요 수정 내역

### 2.1. GUI 카메라 영상 수신 및 출력 구현
*   **문제**: PC GUI 화면에서 카메라 피드가 나타나지 않음. 로봇(`pinky_core`) 측에 카메라 드라이버 인터페이스만 있고 실제 이미지를 캡처하여 TCP로 전송하는 구현체가 부재함.
*   **수정**:
    *   `pinky_core/include/pinky_core/hal/opencv_camera.h` 및 `pinky_core/src/hal/opencv_camera.cpp` 신규 작성.
    *   OpenCV(`cv::VideoCapture`)를 활용하여 카메라(디바이스 0)로부터 프레임을 가져오도록 구현.
    *   기존 파이썬 환경(`pinkylib`)과 하드웨어 방향을 맞추기 위해 영상을 180도 회전(`cv::ROTATE_180`) 적용.
    *   JPEG 압축(`cv::imencode`)을 거쳐 `CameraFrame` 메시지로 TCP 브로드캐스팅.
    *   `RobotApp::Init()`에서 `OpencvCamera` 초기화 연결 및 `CMakeLists.txt` 빌드 소스 추가.

### 2.2. GUI 라이다(LiDAR) 2D 스캔 그래프 출력 에러 수정
*   **문제**: PyQtGraph의 ScatterPlotItem을 사용하는 과정에서 인자 전달 타입 문제로 인해 그래프 뷰가 비어있는 현상 발생.
*   **수정**:
    *   `pinky_station/gui/widgets/lidar_view.py` 파일 내 `setData` 호출부 수정.
    *   기존 위치 인자(`self.scatter.setData(x_pts, y_pts)`)에서 명시적 키워드 인자(`self.scatter.setData(x=x_pts, y=y_pts)`)로 변경하여 내부 PyQt6 모듈의 오류 회피.

### 2.3. 맵 위젯(MapWidget) 뷰포트 좌표계 기준 변경
*   **문제**: 로봇이 고정된 상태에서 오도메트리 이동 시 맵의 월드 좌표계 자체가 로봇을 따라다니는 형태로 렌더링되어 사용자가 로봇의 실제 이동 궤적을 시각적으로 파악하기 어려움.
*   **수정**:
    *   `pinky_station/gui/widgets/map_widget.py` 파일의 변환(Transform) 계산 로직 수정.
    *   카메라 뷰포트의 중심이 로봇의 현재 위치(`self.robot_x`, `self.robot_y`)가 아닌 **월드 원점(0, 0)**을 향하도록 변경.
    *   디버깅 편의성을 위해 월드 원점을 표시하는 X/Y(빨강/초록) 고정 축은 유지하고, 로봇이 이 원점을 기준으로 오도메트리 이동량만큼 캔버스 상에서 움직이도록(Absolute coordinate rendering) 개선.

### 2.4. 원격 조작(Teleop) 속도 조절 분리
*   **문제**: 기존 Teleop 제어 위젯에 속도 조절 슬라이더가 1개뿐이어서 전진(Linear)과 회전(Angular) 속도를 독립적으로 제어할 수 없음.
*   **수정**:
    *   `pinky_station/gui/widgets/teleop_widget.py` 파일 수정.
    *   병진 속도 조절용 `L-Speed` 슬라이더와 회전 속도 조절용 `A-Speed` 슬라이더를 분리.
    *   회전 속도(Angular Velocity)는 0.1 ~ 3.0 rad/s 범위로 설정 가능하도록 구현.
    *   방향키 입력 시 각 슬라이더에 설정된 독립적인 속도값이 프로토콜로 전송되도록 이벤트 매핑 갱신.

### 2.5. BNO055 IMU 통신 실패 시 시스템 프리징(Freezing) 방지
*   **문제**: 하드웨어 결함 또는 선 빠짐 등으로 BNO055 I2C 칩 ID(`0xA0`)를 정상적으로 읽지 못하는 상황에서도 `Bno055Imu::Init()` 내부의 통신 재시도 루프(while 문)로 진입하여 스레드가 블로킹됨. 이로 인해 터미널이 `Ctrl+C` 입력에 응답하지 않는 데드락 발생.
*   **수정**:
    *   `pinky_core/src/hal/bno055_imu.cpp` 파일 수정.
    *   `chipid`가 `0xA0`가 아닐 경우 경고 로그 출력 직후 즉시 `return false;` (Early Return) 하도록 로직 변경.
    *   하드웨어 오류 시 I2C 설정만 실패 처리하고 로봇 메인 프로세스는 멈춤 없이 다른 센서 및 구동 로직을 이어가도록 시스템 안정성 확보.

## 3. 결과 및 향후 계획
*   **검증**: 위 모든 수정 사항은 로컬 PC 상에서 단위 테스트(`pytest tests/`) 및 `make -j4` 빌드를 통과하였으며 동작 확인을 완료함.
*   **향후 계획**: 
    *   카메라 영상의 프레임 레이트 및 해상도 최적화.
    *   GUI 리팩토링 안정화 및 통합 시뮬레이션 환경(Gazebo)과의 오도메트리 동기화 테스트.