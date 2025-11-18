import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 6000))
print("🤖 Sẵn sàng nhận lệnh từ laptop...")

while True:
    data, addr = sock.recvfrom(1024)
    cmd = data.decode()
    print(f"📥 Nhận lệnh: {cmd}")
    # TODO: Điều khiển động cơ ở đây (GPIO / UART / ESP32...)
