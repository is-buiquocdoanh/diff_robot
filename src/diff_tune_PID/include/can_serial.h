#ifndef CAN_SERIAL_H
#define CAN_SERIAL_H

#include <Arduino.h>

typedef struct {
  uint32_t id;
  uint8_t data[8];
} DataPacket;

class CanSerial {
  public:
    CanSerial(Stream &serial);

    void begin(unsigned long baudrate);

    bool sendPacket(const DataPacket &packet);
    bool readPacket(DataPacket &packet);

  private:
    Stream *_serial;
    uint8_t _rxBuffer[12];
    uint8_t _rxIndex = 0;
    unsigned long _lastByteMillis = 0;
};

#endif
