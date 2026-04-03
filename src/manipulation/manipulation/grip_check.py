#!/usr/bin/env python3
from pyModbusTCP.client import ModbusClient
import time

class ChickenGripper:
    def __init__(self, ip="192.168.1.1"):
        # 성아님이 찾아낸 Unit ID 65번 적용!
        self.client = ModbusClient(host=ip, port=502, unit_id=65, auto_open=True)
        
        # 2FG7 레지스터 주소 (표준)
        self.ADDR_CONTROL = 1000  # 동작 명령 (1: 시작)
        self.ADDR_FORCE   = 1001  # 힘 설정 (20~140N)
        self.ADDR_WIDTH   = 1002  # 목표 폭 (0.1mm 단위, 200 = 20mm)

    def move(self, width_mm, force_n=40):
        if not self.client.is_open:
            if not self.client.open():
                print("❌ 그리퍼 연결 실패!")
                return False

        # 1. 힘과 폭 설정 (2FG7은 0.1mm 단위이므로 10을 곱함)
        self.client.write_single_register(self.ADDR_FORCE, int(force_n))
        self.client.write_single_register(self.ADDR_WIDTH, int(width_mm * 10))
        
        # 2. 실행 명령 (이걸 써야 움직입니다)
        self.client.write_single_register(self.ADDR_CONTROL, 1)
        
        print(f"✅ 명령 전송: {width_mm}mm / {force_n}N")
        return True

    def get_width(self):
        # 현재 폭 읽기 (보통 1002번 혹은 2002번)
        regs = self.client.read_holding_registers(1002, 1)
        if regs:
            return regs[0] / 10.0
        return None

# 테스트 실행
if __name__ == "__main__":
    gripper = ChickenGripper()
    print("🐥 치킨 그리퍼 테스트를 시작합니다.")
    
    # 50mm로 벌리기
    gripper.move(50)
    time.sleep(2)
    
    # 10mm로 오므리기
    gripper.move(10)
    print(f"현재 위치: {gripper.get_width()} mm")
