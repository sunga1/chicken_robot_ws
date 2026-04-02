#!/usr/bin/env python3
from pyModbusTCP.client import ModbusClient
import time

class ChickenGripperMonitor:
    def __init__(self, ip="192.168.1.1"):
        self.client = ModbusClient(host=ip, port=502, auto_open=True)
        # 2FG7 상태 읽기용 표준 주소 (모델 설정에 따라 다를 수 있음)
        self.ADDR_STATUS = 2000  # 현재 상태 (비트별 의미 다름)
        self.ADDR_WIDTH  = 2002  # 현재 벌어진 폭 (0.1mm 단위)

    def get_status(self):
        if not self.client.is_open:
            self.client.open()

        # 레지스터 읽기 (주소부터 4개 정도 한꺼번에 읽기)
        regs = self.client.read_holding_registers(1000, 10) # Holding 기준
        # 또는
        # regs = self.client.read_input_registers(2000, 10) # Input 기준
        
        if regs:
            # 2FG7 표준 맵 기준 (1000번대 사용 시)
            cur_status = regs[0] # 1000번: Status
            cur_width = regs[2]  # 1002번: Current Width
            
            print(f"--- 그리퍼 상태 모니터링 ---")
            print(f"현재 폭: {cur_width / 10.0} mm")
            print(f"상태 코드: {bin(cur_status)} (이진수)")
            
            # 상태 분석 예시
            if cur_status & 0x0001: print(">> 동작 중 (Busy)")
            if cur_status & 0x0002: print(">> 물체 잡기 성공 (Grip Detected)")
        else:
            print("❌ 데이터를 읽어오지 못했습니다.")

def main():
    monitor = ChickenGripperMonitor("192.168.1.1")
    while True:
        monitor.get_status()
        time.sleep(1) # 1초마다 업데이트

if __name__ == "__main__":
    main()
