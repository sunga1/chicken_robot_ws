#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as R
import sys

def get_quaternion():
    print("\n" + "="*50)
    print("두산 로봇 펜던트 각도(Euler) -> 쿼터니언 변환기")
    print("="*50)
    
    try:
        # 터미널에서 값 입력 받기
        rx = float(input("RX (degree) 입력: "))
        ry = float(input("RY (degree) 입력: "))
        rz = float(input("RZ (degree) 입력: "))

        # 오일러 -> 쿼터니언 변환 (두산 표준 xyz 순서)
        r = R.from_euler('xyz', [rx, ry, rz], degrees=True)
        quat = r.as_quat() # [qx, qy, qz, qw]
        # 소수점 4자리로 반올림
        quat = [round(float(val), 4) for val in quat]

        print("\n" + "-"*50)
        print(f"변환 완료")
        print(f"Euler: RX={rx}, RY={ry}, RZ={rz}")
        print(f"Quaternion [qx, qy, qz, qw]:")
        print(f"   {quat}")
        print("-"*50)
        
        print(f"euler_to_quaternion({rx}, {ry}, {rz})  # 결과: {quat}")
        print("="*50 + "\n")

    except ValueError:
        print("에러: 숫자만 입력해 주세요!")
    except Exception as e:
        print(f"예상치 못한 에러 발생: {e}")

if __name__ == "__main__":
    while True:
        get_quaternion()
        cont = input("계속 변환하시겠습니까? (y/n): ").lower()
        if cont != 'y':
            print("프로그램을 종료")
            break
