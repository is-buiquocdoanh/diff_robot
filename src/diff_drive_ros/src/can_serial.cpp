#include "can_serial.h"

CanSerial::CanSerial(Stream &serial) {
  _serial = &serial;
}

void CanSerial::begin(unsigned long baudrate) {
  if (&Serial == _serial) {
    Serial.begin(baudrate);
  }
}

bool CanSerial::sendPacket(const DataPacket &packet) {
    size_t numByteSend = 0;
    uint8_t header = 0x2a;
    uint8_t tail = 0x23;

    numByteSend += _serial->write(&header, 1);
    // Gửi ID (4 bytes)
    numByteSend += _serial->write((uint8_t*)&packet.id, sizeof(packet.id));
    // Gửi 8 bytes dữ liệu
    numByteSend += _serial->write(packet.data, 8);

    numByteSend += _serial->write(&tail, 1);

    return numByteSend == 14;
}

bool CanSerial::readPacket(DataPacket &packet) {
    // Non-blocking stream parser: look for header 0x2A, collect 12 payload bytes (4 id + 8 data), then tail 0x23
    unsigned long now = millis();
    // if parser has been idle for > 100ms while mid-packet, reset state
    if (_rxIndex != 0 && (now - _lastByteMillis) > 100) {
        _rxIndex = 0;
    }

    while (_serial->available()) {
        int v = _serial->read();
        if (v < 0) break;
        uint8_t data = (uint8_t)v;
        _lastByteMillis = now = millis();

        if (_rxIndex == 0) {
            // waiting for header
            if (data == 0x2A) {
                _rxIndex = 1; // header seen, next bytes are payload
            } else {
                // skip garbage
                continue;
            }
        } else if (_rxIndex >= 1 && _rxIndex <= 12) {
            // store payload bytes into buffer[0..11]
            _rxBuffer[_rxIndex - 1] = data;
            _rxIndex++;
        } else if (_rxIndex == 13) {
            // expecting tail
            if (data == 0x23) {
                // got full packet
                memcpy(&packet.id, _rxBuffer, 4);
                memcpy(packet.data, _rxBuffer + 4, 8);
                _rxIndex = 0;
                return true;
            } else {
                // invalid tail; reset state but treat this byte as possible header
                _rxIndex = 0;
                if (data == 0x2A) {
                    _rxIndex = 1;
                }
                // continue scanning
            }
        } else {
            // should not happen, reset
            _rxIndex = 0;
        }
    }

    return false;
}