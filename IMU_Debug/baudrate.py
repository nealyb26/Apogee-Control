import serial
import time

def calculate_checksum(sentence):
    checksum = 0
    for c in sentence:
        checksum ^= ord(c)
    return f"{checksum:02X}"

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

command_body = "VNWRG,6,2,115200"
checksum = calculate_checksum(command_body)
command = f"${command_body}*{checksum}\r\n"

ser.write(command.encode())
time.sleep(0.5)

ser.close()
# Reopen with new baud rate
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
