# test_send_loop.py
import serial
import struct
import time
import argparse

parser = argparse.ArgumentParser(description="Continuously send CAN-serial test packets to ESP32")
parser.add_argument("--port", default="/dev/ttyUSB1", help="serial port (default /dev/ttyUSB0)")
parser.add_argument("--baud", type=int, default=115200, help="baudrate (default 115200)")
parser.add_argument("--interval", type=float, default=0.1, help="send interval in seconds (default 0.1s)")
parser.add_argument("--lf_dir", type=int, default=2, help="left dir (0/1/2)")
parser.add_argument("--lf_pwm", type=int, default=100, help="left pwm 0..255")
parser.add_argument("--rf_dir", type=int, default=1, help="right dir (0/1/2)")
parser.add_argument("--rf_pwm", type=int, default=50, help="right pwm 0..255")
args = parser.parse_args()

def build_packet(id_val=1, lf_dir=1, lf_pwm=150, rf_dir=1, rf_pwm=60):
    header = b'\x2a'
    tail = b'\x23'
    pid = struct.pack('<I', id_val)
    data = bytearray(8)
    data[0] = lf_dir & 0xFF
    data[1] = lf_pwm & 0xFF
    data[2] = rf_dir & 0xFF
    data[3] = rf_pwm & 0xFF
    # data[4..7] left as 0
    return header + pid + bytes(data) + tail

def main():
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    print(f"Sending to {args.port} @ {args.baud} every {args.interval}s. Ctrl-C to stop.")
    pkt = build_packet(1, args.lf_dir, args.lf_pwm, args.rf_dir, args.rf_pwm)

    try:
        i = 0
        while True:
            ser.write(pkt)
            ser.flush()
            i += 1
            if i % 10 == 0:
                print(f"sent {i} packets  lf={args.lf_pwm}@{args.lf_dir} rf={args.rf_pwm}@{args.rf_dir}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        ser.close()

if __name__ == "__main__":
    main()