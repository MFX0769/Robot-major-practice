import serial
import time

ser = serial.Serial('COM23', 115200, timeout=2)
time.sleep(0.5)

# 尝试各种可能的查询命令
commands = [
    b'AT\r\n',
    b'AT+POWER\r\n',
    b'AT+TXPOWER\r\n',
    b'AT+SETTING\r\n',
    b'AT+READ\r\n',
    b'AT+CONFIG\r\n',
    b'AT+INFO\r\n',
    b'AT+STATUS\r\n',
    b'AT+VERSION\r\n',
    b'\r\n',
    b'?',
    b'HELP\r\n',
]

for cmd in commands:
    ser.write(cmd)
    time.sleep(0.3)
    resp = ser.read(ser.in_waiting or 1024)
    if resp:
        print(f'CMD: {cmd} -> RESP: {resp}')
    else:
        print(f'CMD: {cmd} -> (no response)')

# 也试试直接读取，看有没有主动输出
time.sleep(1)
resp = ser.read(ser.in_waiting or 1024)
if resp:
    print(f'\nPassive output: {resp}')

ser.close()
print('\nDone')
