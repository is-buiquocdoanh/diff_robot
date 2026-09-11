## Kiểm tra thiết bị
#### Cắm từng thiết bị vào để kiểm tra
```
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
```

## Tạo rule
```
sudo nano /etc/udev/rules.d/99-usb-devices.rules
```

### Nội dung file
```
# ESP32
SUBSYSTEM=="tty", KERNELS=="1-2.4.1.3:1.0", SYMLINK+="esp32", MODE="0666"

# Lidar
SUBSYSTEM=="tty", KERNELS=="1-2.1:1.0", SYMLINK+="ydlidar", MODE="0666"
```

## Kích hoạt rule
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

# Kiểm tra
```
ls -l /dev | grep esp32
ls -l /dev | grep ydlidar
```
