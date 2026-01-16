#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "Const.h"

class CANOBD
{
public:
    CANOBD() = default;

    bool send(uint8_t mode, uint8_t pid, bool extended)
    {
        twai_message_t msg = {};
        msg.identifier = extended ? 0x18DB33F1 : 0x7DF;
        msg.extd = extended;
        msg.rtr = 0;
        msg.data_length_code = 8;
        msg.data[0] = 0x02;
        msg.data[1] = mode;
        msg.data[2] = pid;
        for (int i = 3; i < 8; i++)
            msg.data[i] = 0;
        return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
    }

    bool receive(uint8_t *buffer, uint16_t &length, uint32_t timeoutMs)
    {
        uint32_t start = millis();
        while (millis() - start < timeoutMs)
        {
            twai_message_t msg;
            if (twai_receive(&msg, pdMS_TO_TICKS(5)) != ESP_OK)
            {
                vTaskDelay(1);
                continue;
            }

            if (!msg.extd && (msg.identifier < 0x7E8 || msg.identifier > 0x7EF))
                continue;
            if (msg.extd && (msg.identifier & 0x1FFFFF00) != 0x18DAF100)
                continue;

            uint8_t dataLen = msg.data[0] & 0x0F;
            if (dataLen > 7)
                dataLen = 7;

            memcpy(buffer, &msg.data[1], dataLen);
            length = dataLen;
            return true;
        }
        return false;
    }

    uint16_t read_pid(uint8_t mode, uint8_t pid, uint8_t *buffer, uint32_t timeoutMs, bool extended = true)
    {
        uint16_t len = 0;
        if (!send(mode, pid, extended))
            return 0;
        if (!receive(buffer, len, timeoutMs))
            return 0;
        if (len < 2 || buffer[0] != (mode + 0x40) || buffer[1] != pid)
            return 0;
        return len;
    }

    // examples
    float getEngineRPM()
    {
        uint8_t buf[8];
        uint16_t len = read_pid(0x01, 0x0C, buf, 50);
        if (len >= 4)
            return ((buf[2] << 8) | buf[3]) / 4.0f;
        return NAN;
    }

    float getIntakeAirTemp()
    {
        uint8_t buf[8];
        uint16_t len = read_pid(read_LiveData, INTAKE_AIR_TEMP, buf, 100);
        if (len >= 3)
            return (float)buf[2] - 40.0f;
        return NAN;
    }

    float getCoolantTemp()
    {
        uint8_t buf[8];
        uint16_t len = read_pid(read_LiveData, ENGINE_COOLANT_TEMP, buf, 100);
        if (len >= 3)
            return (float)buf[2] - 40.0f;
        return NAN;
    }
};
