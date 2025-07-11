import serial
import time

def calc_checksum(sentence):
    cksum = 0
    for char in sentence:
        cksum ^= ord(char)
    return f"{cksum:02X}"

def send_with_crc(ser, command_body):
    command = f"${command_body}*{calc_checksum(command_body)}\r\n"
    print(f"📤 Sending: {command.strip()}")
    ser.write(command.encode())
    time.sleep(0.2)
    reply = ser.read(ser.in_waiting or 256).decode(errors="replace")
    if reply:
        print(f"🧾 IMU replied: {repr(reply.strip())}")
    else:
        print("⚠️ No reply received.")

def main():
    print("🔌 Connecting to /dev/serial0 @ 115200 baud...")
    with serial.Serial("/dev/serial0", 115200, timeout=1) as ser:
        time.sleep(1.0)

        print("🧨 Sending factory reset command...")
        send_with_crc(ser, "VNRST")  # Factory reset

        print("⏳ Waiting 5 seconds for reboot...")
        time.sleep(5)

        print("📥 Reading IMU output after reset...")
        ser.reset_input_buffer()
        time.sleep(1)
        data = ser.read(256)
        if b"$" in data:
            print("✅ IMU responded in ASCII (default mode). Reset successful.")
        elif data and data[0] == 0xFA:
            print("✅ IMU is in binary mode. Unexpected, but fine.")
        else:
            print(f"⚠️ Unknown or no data after reset: {data[:20]}")

if __name__ == "__main__":
    main()
