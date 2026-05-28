#ifndef _CST816D_H
#define _CST816D_H

#include <Wire.h>

#define I2C_ADDR_CST816D 0x15

enum GESTURE
{
    None       = 0x00,
    SlideDown  = 0x01,
    SlideUp    = 0x02,
    SlideLeft  = 0x03,
    SlideRight = 0x04,
    SingleTap  = 0x05,
    LongPress  = 0x0C
};

class CST816D
{
public:
    CST816D(int8_t sda = -1, int8_t scl = -1, int8_t rst = -1, int8_t intr = -1)
        : _sda(sda), _scl(scl), _rst(rst), _int(intr), _latched(false) {}

    inline void begin()
    {
        if (_sda != -1 && _scl != -1) Wire.begin(_sda, _scl);
        else Wire.begin();

        if (_int != -1)
        {
            pinMode(_int, OUTPUT);
            digitalWrite(_int, HIGH);
            delay(1);
            digitalWrite(_int, LOW);
            delay(1);
        }

        if (_rst != -1)
        {
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, LOW);
            delay(10);
            digitalWrite(_rst, HIGH);
            delay(300);
        }

        write(0xFE, 0xFF);
        _latched = false;
    }

    inline bool getGesture(uint8_t *gesture)
    {
        uint8_t g = read(0x01);

        if (g != SlideUp &&
            g != SlideDown &&
            g != SlideLeft &&
            g != SlideRight &&
            g != SingleTap &&
            g != LongPress)
        {
            _latched = false;
            *gesture = None;
            return false;
        }

        if (_latched)
        {
            *gesture = None;
            return false;
        }

        _latched = true;
        *gesture = g;
        return true;
    }

private:
    int8_t _sda, _scl, _rst, _int;
    bool _latched;

    static inline uint8_t read(uint8_t addr)
    {
        uint8_t data = 0, count;
        do
        {
            Wire.beginTransmission(I2C_ADDR_CST816D);
            Wire.write(addr);
            Wire.endTransmission(false);
            count = Wire.requestFrom(I2C_ADDR_CST816D, 1);
        }
        while (count == 0);

        while (Wire.available()) data = Wire.read();
        return data;
    }

    static inline void write(uint8_t addr, uint8_t data)
    {
        Wire.beginTransmission(I2C_ADDR_CST816D);
        Wire.write(addr);
        Wire.write(data);
        Wire.endTransmission();
    }
};

#endif