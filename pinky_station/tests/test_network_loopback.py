import pytest
import time
import threading
import struct
from pinky_station.net.tcp_client import TcpClient
from pinky_station.net.udp_receiver import UdpReceiver
from pinky_station.protocol import message_types as mt

# [주의] 이 통합 테스트는 C++ 실행 파일 `./pinky_robot` 이 
# 같은 PC(또는 보드) 백그라운드에서 실행 중이라는 가정하에 동작합니다!

@pytest.fixture
def network_clients():
    tcp = TcpClient(host="127.0.0.1", port=9100)
    udp = UdpReceiver(port=9200)
    
    tcp.connect()
    udp.start()
    
    # 서버가 클라이언트 IP를 인식하고 UDP 타겟을 세팅할 시간 확보
    time.sleep(0.5) 
    
    yield tcp, udp
    
    tcp.disconnect()
    udp.stop()

def test_tcp_ping_pong_integration(network_clients):
    tcp, udp = network_clients
    assert tcp.is_connected() == True

    # 1. Ping 메시지 (C++에서 수신 여부는 C++ 터미널 로그로 확인 가능)
    # 현재 C++ 코어(robot_app.cpp)는 ping을 받으면 last_ping을 갱신합니다.
    current_time_ms = int(time.time() * 1000)
    payload = struct.pack('<Q', current_time_ms)
    
    # TCP 클라이언트를 통해 Raw 메시지 전송 (send_message 래퍼가 알아서 헤더/Crc 부착 기대)
    success = tcp.send_message(mt.MSG_PING, payload)
    assert success == True

def test_udp_streaming_integration(network_clients):
    tcp, udp = network_clients
    
    received_msgs = []
    
    # 콜백 등록
    def on_udp_msg(msg_type, payload):
        received_msgs.append((msg_type, payload))
        
    udp.set_callback(on_udp_msg)
    
    # 데이터가 날아올 때까지 대기
    time.sleep(1.0)
    
    # C++ 로봇은 최소한 Odom(50Hz), Imu(100Hz), Battery(20Hz), Lidar(10Hz)를 쏴야 합니다.
    assert len(received_msgs) > 0, "C++ 서버로부터 수신된 패킷이 하나도 없습니다! (직렬화 이중 프레이밍 버그 의심)"
    
    # 어떤 타입들이 수신되었는지 분류
    types_received = {msg[0] for msg in received_msgs}
    
    print(f"\n[수신된 메시지 종류들]: {types_received}")
    
    # ODOM 수신 확인 (msg_type 0x12)
    assert mt.MSG_ODOM in types_received, "ODOM 패킷 수신 실패. SerializeOdom() 프레이밍 구조를 확인해야 합니다."
