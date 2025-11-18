import os
import socket
from google.cloud import speech

# --- Cấu hình Google Cloud ---
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/path/to/key.json"
client = speech.SpeechClient()

# --- Socket nhận audio từ Pi ---
IP = "0.0.0.0"
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((IP, PORT))
print("🧠 Laptop đang nhận dữ liệu âm thanh...")

# --- Socket gửi phản hồi điều khiển lại cho Pi ---
PI_IP = "192.168.1.50"
PI_PORT = 6000
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Hàm nhận dạng ---
def recognize_audio(data):
    audio = speech.RecognitionAudio(content=data)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="vi-VN"
    )
    response = client.recognize(config=config, audio=audio)
    for result in response.results:
        text = result.alternatives[0].transcript.lower()
        print(f"🎧 Nghe thấy: {text}")

        # Phân tích lệnh
        if "tiến" in text:
            cmd = "FORWARD"
        elif "lùi" in text:
            cmd = "BACKWARD"
        elif "trái" in text:
            cmd = "LEFT"
        elif "phải" in text:
            cmd = "RIGHT"
        elif "dừng" in text:
            cmd = "STOP"
        else:
            cmd = "NONE"

        # Gửi lệnh về Pi
        cmd_sock.sendto(cmd.encode(), (PI_IP, PI_PORT))
        print(f"➡️  Gửi lệnh: {cmd}")

# --- Nhận và xử lý gói âm thanh ---
buffer = b""
while True:
    data, addr = sock.recvfrom(4096)
    buffer += data
    if len(buffer) > 16000 * 2:  # khoảng 1 giây âm thanh
        recognize_audio(buffer)
        buffer = b""
