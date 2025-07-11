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
    time.sleep(0.3)
    reply = ser.read(ser.in_waiting or 512).decode(errors="replace")
    if reply:
        print(f"🧾 IMU replied: {repr(reply.strip())}")
    else:
        print("⚠️ No reply from IMU.")

def configure_port(ser, port_num):
    print(f"\n--- Testing binary config on Port {port_num} ---")
    # Minimal valid binary config command:
    command = f"VNWRG,75,{port_num},4,01,0007,0000"
    send_with_crc(ser, command)
    send_with_crc(ser, "VNSAV")
    time.sleep(1)
    ser.reset_input_buffer()
    time.sleep(0.5)
    data = ser.read(100)
    if data and data[0] == 0xFA:
        print(f"✅ IMU now outputting binary on Port {port_num}.")
        return True
    else:
        print(f"❌ IMU did not switch to binary mode on Port {port_num}.")
        print(f"Received raw data: {data[:20]}")
        return False

def main():
    # Change this to the serial port connected to your IMU (e.g., '/dev/serial0' or '/dev/ttyAMA0')
    serial_port = "/dev/serial0"
    baudrate = 115200

    print(f"🔌 Connecting to {serial_port} at {baudrate} baud...")
    with serial.Serial(serial_port, baudrate, timeout=1) as ser:
        time.sleep(1)

        # Detect initial mode (ASCII or binary)
        ser.reset_input_buffer()
        data = ser.read(100)
        if b"$" in data:
            print("🟢 Detected ASCII output before config.")
        elif data and data[0] == 0xFA:
            print("🟢 Detected binary output before config.")
        else:
            print("🟡 Could not detect output format before config.")

        # Test Port 1
        port1_success = configure_port(ser, 1)

        # Wait and flush before next test
        time.sleep(2)
        ser.reset_input_buffer()

        # Test Port 2
        port2_success = configure_port(ser, 2)

        if not port1_success and not port2_success:
            print("\n⚠️ IMU did not accept binary config on either Port 1 or Port 2.")
            print("Please check wiring, IMU settings, or try VectorNav Sensor Explorer.")

if __name__ == "__main__":
    main()
