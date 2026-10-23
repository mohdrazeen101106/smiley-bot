#pragma once
#include <Arduino.h>

// EXAMPLE GPIOs only. Replace after wiring. One BTS7960/IBT-2 module per motor.
// Each module has RPWM, LPWM, R_EN, L_EN.
struct MotorPins {
    uint8_t rpwm;
    uint8_t lpwm;
    const char *name;
};

// Order is [front-left, front-right, rear-left, rear-right].
// Do not use these GPIOs for another peripheral. All are valid PWM-capable RP2040 GPIOs.
static constexpr MotorPins MOTOR_PINS[4] = {
    {2,  3, "FL"},
    {6,  7, "FR"},
    {10, 11, "RL"},
    {14, 15, "RR"},
};

// Standard X-drive mixer. Input is [vx=forward, vy=left, vw=CCW rotation].
// Output order [FL, FR, RL, RR]. This presumes the wheel rollers are arranged
// for an X-drive. If the actual mechanical orientation differs, revise this matrix.
static constexpr float MOTOR_MIXING_MATRIX[4][3] = {
    {+1.0f, -1.0f, -1.0f}, // FL
    {+1.0f, +1.0f, +1.0f}, // FR
    {+1.0f, +1.0f, -1.0f}, // RL
    {+1.0f, -1.0f, +1.0f}, // RR
};

// Electrical reversal correction per motor; set to -1 only when testing proves needed.
static constexpr float MOTOR_POLARITY[4] = {+1.0f, +1.0f, +1.0f, +1.0f};

static constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
static constexpr uint8_t PWM_RESOLUTION = 255;
static constexpr float MAX_PWM_DUTY_FRACTION = 0.35f; // conservative starting cap
static constexpr float INPUT_DEADBAND = 0.015f;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 250;
static constexpr uint32_t DIRECTION_CHANGE_DEADTIME_MS = 10;
