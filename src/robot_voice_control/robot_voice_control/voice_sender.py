import socket
import pyaudio

SERVER_IP = "192.168.1.10"  # IP của Laptop
PORT = 5000

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,
                input=True, frames_per_buffer=1024)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("🎤 Bắt đầu gửi dữ liệu âm thanh...")

while True:
    data = stream.read(1024, exception_on_overflow=False)
    sock.sendto(data, (SERVER_IP, PORT))
    print("Đã gửi dữ liệu âm thanh.")