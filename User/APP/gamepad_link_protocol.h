#ifndef GAMEPAD_LINK_PROTOCOL_H
#define GAMEPAD_LINK_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define GAMEPAD_LINK_SOF_0                         0xA5U
#define GAMEPAD_LINK_SOF_1                         0x5AU
#define GAMEPAD_LINK_VERSION                       0x01U
#define GAMEPAD_LINK_FRAME_SIZE                    12U

#define GAMEPAD_LINK_INDEX_SOF_0                   0U
#define GAMEPAD_LINK_INDEX_SOF_1                   1U
#define GAMEPAD_LINK_INDEX_VERSION                 2U
#define GAMEPAD_LINK_INDEX_SEQUENCE                3U
#define GAMEPAD_LINK_INDEX_BUTTONS_LOW             4U
#define GAMEPAD_LINK_INDEX_BUTTONS_HIGH            5U
#define GAMEPAD_LINK_INDEX_LX                      6U
#define GAMEPAD_LINK_INDEX_LY                      7U
#define GAMEPAD_LINK_INDEX_RX                      8U
#define GAMEPAD_LINK_INDEX_RY                      9U
#define GAMEPAD_LINK_INDEX_STATUS                 10U
#define GAMEPAD_LINK_INDEX_CRC                    11U

#define GAMEPAD_LINK_STATUS_CONTROLLER_CONNECTED  (1U << 0)

#define GAMEPAD_BUTTON_A                           (1U << 0)
#define GAMEPAD_BUTTON_B                           (1U << 1)
#define GAMEPAD_BUTTON_X                           (1U << 2)
#define GAMEPAD_BUTTON_Y                           (1U << 3)
#define GAMEPAD_BUTTON_LB                          (1U << 4)
#define GAMEPAD_BUTTON_RB                          (1U << 5)
#define GAMEPAD_BUTTON_SELECT                      (1U << 6)
#define GAMEPAD_BUTTON_START                       (1U << 7)
#define GAMEPAD_BUTTON_XBOX                        (1U << 8)
#define GAMEPAD_BUTTON_SHARE                       (1U << 9)
#define GAMEPAD_BUTTON_LS                          (1U << 10)
#define GAMEPAD_BUTTON_RS                          (1U << 11)
#define GAMEPAD_BUTTON_DIR_UP                      (1U << 12)
#define GAMEPAD_BUTTON_DIR_RIGHT                   (1U << 13)
#define GAMEPAD_BUTTON_DIR_DOWN                    (1U << 14)
#define GAMEPAD_BUTTON_DIR_LEFT                    (1U << 15)

static inline uint8_t GamepadLink_Crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0U;
    size_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U)
                                : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static inline uint8_t GamepadLink_FrameIsValid(const uint8_t *frame)
{
    if ((frame[GAMEPAD_LINK_INDEX_SOF_0] != GAMEPAD_LINK_SOF_0) ||
        (frame[GAMEPAD_LINK_INDEX_SOF_1] != GAMEPAD_LINK_SOF_1) ||
        (frame[GAMEPAD_LINK_INDEX_VERSION] != GAMEPAD_LINK_VERSION))
    {
        return 0U;
    }

    return (GamepadLink_Crc8(frame, GAMEPAD_LINK_INDEX_CRC) ==
            frame[GAMEPAD_LINK_INDEX_CRC]) ? 1U : 0U;
}

#endif
