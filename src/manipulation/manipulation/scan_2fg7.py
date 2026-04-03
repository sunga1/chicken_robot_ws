from pymodbus.client import ModbusTcpClient
import time

IP = "192.168.1.1"
PORT = 502
UNIT = 65   # OnRobot 기본 device id

client = ModbusTcpClient(IP, port=PORT)

print("Connecting...")
client.connect()

def grip_close():
    width = 10
    force = 40
    speed = 50

    values = [int(width*10), force, speed, 1]

    result = client.write_registers(
        address=0,
        values=values,
        device_id=UNIT
    )

    print(result)

def grip_open():
    width = 70
    force = 20
    speed = 50

    values = [int(width*10), force, speed, 1]

    result = client.write_registers(
        address=0,
        values=values,
        device_id=UNIT
    )

    print(result)

print("close")
grip_close()

time.sleep(3)

print("open")
grip_open()

client.close()
