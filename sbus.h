#pragma once
#include <Arduino.h>

constexpr int      SBUS_NUM_CHANNELS    = 16;
constexpr uint16_t CRSF_CHANNEL_MID     = 992;   // Center value (CRSF range 191-1792)
constexpr uint8_t  SBUS_HEADER          = 0x0F;
constexpr uint8_t  SBUS_FOOTER          = 0x00;
constexpr uint8_t  SBUS_FLAG_SIGLOSS    = (1 << 2);
constexpr uint8_t  SBUS_FLAG_FAILSAFE   = (1 << 3);

// 16 x 11-bit channels packed into 22 bytes — standard SBUS layout
typedef struct __attribute__((packed)) {
    unsigned ch0  : 11; unsigned ch1  : 11; unsigned ch2  : 11; unsigned ch3  : 11;
    unsigned ch4  : 11; unsigned ch5  : 11; unsigned ch6  : 11; unsigned ch7  : 11;
    unsigned ch8  : 11; unsigned ch9  : 11; unsigned ch10 : 11; unsigned ch11 : 11;
    unsigned ch12 : 11; unsigned ch13 : 11; unsigned ch14 : 11; unsigned ch15 : 11;
} sbus_channels_t;

// Pack channel array into SBUS bitfield struct and write a 25-byte frame
inline void sbusWrite(HardwareSerial &port, const uint16_t *ch, uint8_t flags) {
    sbus_channels_t p;
    p.ch0  = ch[0];  p.ch1  = ch[1];  p.ch2  = ch[2];  p.ch3  = ch[3];
    p.ch4  = ch[4];  p.ch5  = ch[5];  p.ch6  = ch[6];  p.ch7  = ch[7];
    p.ch8  = ch[8];  p.ch9  = ch[9];  p.ch10 = ch[10]; p.ch11 = ch[11];
    p.ch12 = ch[12]; p.ch13 = ch[13]; p.ch14 = ch[14]; p.ch15 = ch[15];

    port.write(SBUS_HEADER);
    port.write((const uint8_t *)&p, sizeof(p));
    port.write(flags);
    port.write(SBUS_FOOTER);
}
