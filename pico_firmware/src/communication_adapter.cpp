#include "communication_adapter.h"
#include <cstring>
#include <cstdint>
#include <limits>

namespace {
constexpr uint8_t BLITZ_HEADER = 0xAA;
constexpr uint8_t BLITZ_VELOCITY_ID = 5;
constexpr size_t PAYLOAD_SIZE = sizeof(float) * 3;
constexpr size_t FRAME_SIZE = 2 + PAYLOAD_SIZE;
static_assert(sizeof(float) == 4, "Blitz Velocity requires 32-bit float fields");
static_assert(std::numeric_limits<float>::is_iec559, "Blitz Velocity requires IEEE-754 floats");
static_assert(sizeof(VelocityCommandFrame) == PAYLOAD_SIZE, "Unexpected Velocity struct padding");

uint8_t rxFrame[FRAME_SIZE]{};
size_t rxLength = 0;
}

void communicationBegin() {
    Serial.begin(115200); // USB CDC on original Raspberry Pi Pico
    rxLength = 0;
}

bool communicationPoll(VelocityCommandFrame &frame) {
    while (Serial.available() > 0) {
        int incoming = Serial.read();
        if (incoming < 0) break;
        uint8_t value = static_cast<uint8_t>(incoming);

        if (rxLength == 0) {
            if (value == BLITZ_HEADER) {
                rxFrame[0] = value;
                rxLength = 1;
            }
            continue;
        }

        if (rxLength == 1) {
            if (value == BLITZ_VELOCITY_ID) {
                rxFrame[1] = value;
                rxLength = 2;
            } else if (value == BLITZ_HEADER) {
                // Keep the latest candidate header while resynchronizing.
                rxFrame[0] = BLITZ_HEADER;
                rxLength = 1;
            } else {
                rxLength = 0;
            }
            continue;
        }

        rxFrame[rxLength++] = value;
        if (rxLength == FRAME_SIZE) {
            // Arduino-Pico's RP2040 target is little-endian, matching '<BBfff'.
            // Copy bytes rather than doing unaligned float pointer casts.
            memcpy(&frame.vx, &rxFrame[2], sizeof(float));
            memcpy(&frame.vy, &rxFrame[2 + sizeof(float)], sizeof(float));
            memcpy(&frame.vw, &rxFrame[2 + 2 * sizeof(float)], sizeof(float));
            rxLength = 0;
            return true;
        }
    }
    return false;
}
