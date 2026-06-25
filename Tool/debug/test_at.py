import serial
import time

ser = serial.Serial('COM23', 115200, timeout=2)
time.sleep(0.5)

commands = [
    'AT',
    'AT+VERSION',
    'AT+POWER?',
    'AT+TXPOWER?',
    'AT+HELP',
    'AT?',
    'AT+READ?',
    'AT+WRITE?',
]

for cmd in commands:
    ser.write((cmd + '\r\n').encode())
    time.sleep(0.3)
    resp = ser.read(ser.in_waiting or 1024)
    text = resp.decode('utf-8', errors='ignore').strip()
    print(f'{cmd} -> {text}')

ser.close()
