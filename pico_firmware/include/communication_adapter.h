#pragma once
#include <Arduino.h>

// Wire contract from the untouched Robocon r2/blitz_drive package:
// [0xAA][0x05][vx float32 LE][vy float32 LE][vw float32 LE], exactly 14 bytes.
// It is Blitz.pack()'s '<BBfff' packet for Velocity (ID 5). No checksum exists
// in the upstream protocol; this adapter deliberately does not alter the format.
struct VelocityCommandFrame {
    float vx;
    float vy;
    float vw;
};

void communicationBegin();
// True once for each newly received complete packet ID 5; does not repeat stale data.
bool communicationPoll(VelocityCommandFrame &frame);
