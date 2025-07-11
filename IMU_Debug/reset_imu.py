import serial
import time

def calc_checksum(sentence):
    cksum = 0
    for char in sentence:
        cksum ^= ord(char)
    return f"{cksum:02X}"

def send_with_crc(ser, command_body):
    command = f"${command_body}*{calc_checksum(command_body)}\r\n"
    ser.write(command.encode())
    time.sleep(0.2)
    print(f"📤 Sent: {command.strip()}")
    reply = ser.read(ser.in_waiting or 512).decode(errors="replace")
    if reply:
        print(f"🧾 IMU replied: {repr(reply.strip())}")
    else:
        print("⚠️ No reply from IMU.")

def detect_ascii_or_binary(ser):
    print("🔍 Checking if IMU is in ASCII or binary mode...")
    time.sleep(0.5)
    ser.reset_input_buffer()
    data = ser.read(100)
    
    if not data:
        print("⚠️ No data received. IMU might not be connected or active.")
        return None

    if b"$" in data:
        print("🟢 Detected ASCII output (e.g. $VNYMR,...).")
        return "ascii"
    elif data[0] == 0xFA:
        print("🟢 Detected binary output (starts with 0xFA).")
        return "binary"
    else:
        print(f"🟡 Received unknown data format: {data[:10]}")
        return "unknown"

def configure_binary_output(ser):
    print("🔧 Sending binary config to IMU...")

    # Minimal valid binary config: YPR + Accel only on Port 2 at 200 Hz
    send_with_crc(ser, "VNWRG,75,2,4,01,0007,0000")
    send_with_crc(ser, "VNSAV")

    print("⏳ Waiting 1 second to let changes take effect...")
    time.sleep(1)

    ser.reset_input_buffer()
    time.sleep(0.5)
    data = ser.read(100)
    if data and data[0] == 0xFA:
        print("✅ IMU now outputting binary.")
        return True
    else:
        print("❌ IMU is still not outputting binary.")
        return False

def main():
    print("🔌 Connecting to /dev/serial0 at 115200 baud...")
    with serial.Serial("/dev/serial0", 115200, timeout=1) as ser:
        time.sleep(1.0)

        mode = detect_ascii_or_binary(ser)

        if mode == "binary":
            print("✅ IMU is already in binary mode. No changes made.")
        elif mode == "ascii":
            print("⚙️ Switching IMU to binary mode...")
            success = configure_binary_output(ser)
            if not success:
                print("⚠️ Failed to enable binary mode. Try checking connections or using VN Sensor Explorer.")
        else:
            print("❌ Unknown IMU output. Could not identify mode.")

if __name__ == "__main__":
    main()
