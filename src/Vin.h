// For a semi-universal function to get the VIN, these must be implemented (according to commaai/opendbc):
// - GM VIN request, Bolt FWD Camera
// - KWP VIN request, Nissan Leaf VCM
// - UDS VIN request, Volkswagen FWD Camera
// - UDS VIN request, Rivian EPAS

/* TODO:
    - Get Make, model, and trim:
        Ping ECUs for firmware version/calibration ID + ECUs that only exist on certain trims

    - Get DTC/maintence codes
*/

/* 
    The idea for this code was taken from vin.py in the comma.ai/opendbc github repository.
    They have a retry feature, as well as propriety commands for other cars (will be added eventually)
    This code is slightly different, as it sends both the extended and non-extended requests 
    for the OBD vommands, which may be faster but could be "bus spam" and slow down/reject 
    responses on some cars. UDS sends directly to the functional address.
*/

#pragma once
#include <cstdint>
#include "driver/twai.h"

class Vin
{
public:
    Vin() = default;

    const char *get_vin(int attempts = 1)
    {
        static char vin[18];
        for (int i = 0; i < attempts; i++)
        {
            if (get_vin_uds(vin, attempts))
                return vin;

            if (get_vin_obd(vin, attempts))
                return vin;
        }

        vin[0] = 0;
        return vin;
    }

private:
    void check_vin(char *vin)
    {
        if (vin[0] == 0x11) // Honda Bosch starts with padding, shift VIN
        {
            for (int i = 0; i < 16; i++)
                vin[i] = vin[i + 1];
            vin[16] = 0;
        }

        size_t len = strlen(vin);
        if (len == 19 || len == 24) // VIN can be 19 or 24 bytes long
        {
            while (len > 0 && vin[len - 1] == 0x00) // Remove padding (Ford/Nissan)
                vin[--len] = 0;
        }
    }
    bool get_vin_uds(char *vin_out, int attempt)
    {
        int vin_index = 0;
        uint8_t seq_num = 1;
        bool first_frame = false;

        twai_message_t tx = {};
        tx.identifier = 0x18DB33F1;
        tx.extd = 1;
        tx.data_length_code = 8;
        tx.data[0] = 0x03;
        tx.data[1] = 0x22;
        tx.data[2] = 0xF1;
        tx.data[3] = 0x90;
        twai_transmit(&tx, pdMS_TO_TICKS(20));

        uint32_t start = millis();
        uint32_t timeout = 1500 + attempt * 500;

        while (millis() - start < timeout)
        {
            twai_message_t msg;
            if (twai_receive(&msg, pdMS_TO_TICKS(25)) != ESP_OK)
                continue;

            if (!msg.extd)
                continue;

            uint8_t pci = msg.data[0] >> 4;

            if (pci == 0x0 &&
                msg.data[1] == 0x49 &&
                msg.data[2] == 0x02)
            {
                for (int i = 3; i < msg.data_length_code && vin_index < 17; i++)
                    vin_out[vin_index++] = msg.data[i];

                vin_out[vin_index] = 0;
                check_vin(vin_out);
                return true;
            }

            if (pci == 0x1 &&
                msg.data[2] == 0x62 &&
                msg.data[3] == 0xF1 &&
                msg.data[4] == 0x90)
            {
                first_frame = true;
                vin_index = 0;
                seq_num = 1;

                for (int i = 6; i < 8 && vin_index < 17; i++)
                    vin_out[vin_index++] = msg.data[i];

                twai_message_t fc = {};
                fc.identifier = 0x18DB33F1;
                fc.extd = 1;
                fc.data_length_code = 8;
                fc.data[0] = 0x30;
                twai_transmit(&fc, pdMS_TO_TICKS(5));
            }
            else if (pci == 0x2 && first_frame)
            {
                if ((msg.data[0] & 0x0F) != seq_num)
                    continue;

                seq_num = (seq_num + 1) & 0x0F;

                for (int i = 1; i < 8 && vin_index < 17; i++)
                    vin_out[vin_index++] = msg.data[i];

                if (vin_index >= 17)
                {
                    vin_out[vin_index] = 0;
                    check_vin(vin_out);
                    return true;
                }
            }
        }

        return false;
    }

    bool get_vin_obd(char *vin_out, int attempt)
    {
        int vin_index = 0;
        uint32_t ecu_rx_id = 0;
        bool extd_ecu = false;
        bool first_frame = false;
        uint8_t seq = 1;

        constexpr uint32_t ADDRS[] = {0x7DF, 0x18DB33F1};

        for (uint32_t id : ADDRS)
        { // Sends both requests to speed up response. Might not be good because of bus spam
            twai_message_t tx = {};
            tx.identifier = id;
            tx.extd = (id > 0x7FF);
            tx.data_length_code = 8;
            tx.data[0] = 0x02;
            tx.data[1] = read_VehicleInfo;
            tx.data[2] = read_VIN;
            twai_transmit(&tx, pdMS_TO_TICKS(20));
        }

        uint32_t start = millis();
        uint32_t timeout = 1500 + attempt * 500;

        while (millis() - start < timeout)
        {
            twai_message_t msg;
            if (twai_receive(&msg, pdMS_TO_TICKS(25)) != ESP_OK)
                continue;

            uint8_t pci = msg.data[0] >> 4;

            if (pci == 0x0 &&
                msg.data[1] == 0x49 &&
                msg.data[2] == 0x02)
            {
                for (int i = 3; i < msg.data_length_code && vin_index < 17; i++)
                    vin_out[vin_index++] = msg.data[i];

                vin_out[vin_index] = 0;
                check_vin(vin_out);
                return true;
            }

            if (pci == 0x1 &&
                msg.data[2] == 0x49 &&
                msg.data[3] == 0x02)
            {
                ecu_rx_id = msg.identifier;
                extd_ecu = msg.extd;
                first_frame = true;
                seq = 1;
                vin_index = 0;

                for (int i = 5; i < 8 && vin_index < 17; i++)
                    vin_out[vin_index++] = msg.data[i];

                twai_message_t fc = {};
                fc.extd = extd_ecu;
                fc.data_length_code = 8;
                fc.data[0] = 0x30;
                fc.identifier = extd_ecu
                                    ? (0x18DA0000 | ((ecu_rx_id & 0xFF) << 8) | ((ecu_rx_id >> 8) & 0xFF))
                                    : (ecu_rx_id - 8);

                twai_transmit(&fc, pdMS_TO_TICKS(5));
            }
            else if (pci == 0x2 && first_frame && msg.identifier == ecu_rx_id)
            {
                if ((msg.data[0] & 0x0F) != seq)
                    continue;

                seq = (seq + 1) & 0x0F;

                for (int i = 1; i < msg.data_length_code && vin_index < 17; i++)
                    vin_out[vin_index++] = msg.data[i];

                if (vin_index >= 17)
                {
                    vin_out[vin_index] = 0;
                    check_vin(vin_out);
                    return true;
                }
            }
        }

        return false;
    }

    // inline const char (*get_dtc_list(uint8_t &count))[6]{} // can't work on until next maintence code pops up lol
};
