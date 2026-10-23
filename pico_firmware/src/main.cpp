#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include "motor_config.h"
#include "communication_adapter.h"

struct MotorRuntime {
    int8_t activeDirection = 0;  // -1 reverse, 0 stopped/braking, +1 forward
    int8_t pendingDirection = 0;
    bool reversingWait = false;
    uint32_t reversingStartedAt = 0;
};

static MotorRuntime motorState[4];
static VelocityCommandFrame latestVelocity{0.0f, 0.0f, 0.0f};
static uint32_t lastValidCommandMs = 0;
static bool haveValidCommand = false;
static uint32_t invalidFrameCount = 0;

static void enableDriver(const MotorPins &pins) {
    digitalWrite(pins.r_en, HIGH);
    digitalWrite(pins.l_en, HIGH);
}

static void setPwmLow(const MotorPins &pins) {
    analogWrite(pins.rpwm, 0);
    analogWrite(pins.lpwm, 0);
}

// Active braking for common BTS7960/IBT-2 boards: both enables HIGH and both
// PWM inputs LOW. Verify this against the exact module/clone before motor power.
static void brakeMotor(size_t index) {
    const MotorPins &pins = MOTOR_PINS[index];
    setPwmLow(pins);
    enableDriver(pins);
    motorState[index].activeDirection = 0;
    motorState[index].pendingDirection = 0;
    motorState[index].reversingWait = false;
}

static void brakeAll() {
    for (size_t i = 0; i < 4; ++i) brakeMotor(i);
}

static bool velocityIsValid(const VelocityCommandFrame &v) {
    return isfinite(v.vx) && isfinite(v.vy) && isfinite(v.vw)
        && fabsf(v.vx) <= 1.0f
        && fabsf(v.vy) <= 1.0f
        && fabsf(v.vw) <= 1.0f;
}

static void mixVelocity(const VelocityCommandFrame &v, float wheel[4]) {
    const float input[3] = {v.vx, v.vy, v.vw};
    float peak = 1.0f;
    for (size_t i = 0; i < 4; ++i) {
        float mixed = 0.0f;
        for (size_t j = 0; j < 3; ++j) {
            mixed += MOTOR_MIXING_MATRIX[i][j] * input[j];
        }
        wheel[i] = mixed * MOTOR_POLARITY[i];
        const float mag = fabsf(wheel[i]);
        if (mag > peak) peak = mag;
    }
    // Preserve relative wheel proportions during combined/diagonal input.
    if (peak > 1.0f) {
        for (size_t i = 0; i < 4; ++i) wheel[i] /= peak;
    }
}

static uint8_t commandToDuty(float value) {
    float magnitude = fabsf(value);
    if (magnitude > 1.0f) magnitude = 1.0f;
    const float limited = magnitude * MAX_PWM_DUTY_FRACTION;
    return static_cast<uint8_t>(lroundf(limited * PWM_RESOLUTION));
}

static void driveInDirection(size_t index, int8_t direction, uint8_t duty) {
    const MotorPins &pins = MOTOR_PINS[index];
    enableDriver(pins);
    if (direction > 0) {
        analogWrite(pins.lpwm, 0);
        analogWrite(pins.rpwm, duty);
    } else {
        analogWrite(pins.rpwm, 0);
        analogWrite(pins.lpwm, duty);
    }
    motorState[index].activeDirection = direction;
}

static void applyMotorCommand(size_t index, float command, uint32_t now) {
    if (!isfinite(command) || fabsf(command) <= INPUT_DEADBAND) {
        brakeMotor(index);
        return;
    }

    const int8_t requestedDirection = command > 0.0f ? 1 : -1;
    MotorRuntime &state = motorState[index];

    if (state.reversingWait) {
        if (requestedDirection != state.pendingDirection) {
            state.pendingDirection = requestedDirection;
            state.reversingStartedAt = now;
        }
        setPwmLow(MOTOR_PINS[index]);
        enableDriver(MOTOR_PINS[index]);
        if (static_cast<uint32_t>(now - state.reversingStartedAt) < DIRECTION_CHANGE_DEADTIME_MS) {
            return;
        }
        state.reversingWait = false;
        state.activeDirection = 0;
    }

    if (state.activeDirection != 0 && requestedDirection != state.activeDirection) {
        state.reversingWait = true;
        state.pendingDirection = requestedDirection;
        state.reversingStartedAt = now;
        setPwmLow(MOTOR_PINS[index]);
        enableDriver(MOTOR_PINS[index]);
        return;
    }

    driveInDirection(index, requestedDirection, commandToDuty(command));
}

static void applyVelocity() {
    float wheel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    mixVelocity(latestVelocity, wheel);
    const uint32_t now = millis();
    for (size_t i = 0; i < 4; ++i) applyMotorCommand(i, wheel[i], now);
}

void setup() {
    // Keep the H-bridges disabled until GPIO/PWM is initialized.
    for (size_t i = 0; i < 4; ++i) {
        const MotorPins &pins = MOTOR_PINS[i];
        pinMode(pins.rpwm, OUTPUT);
        pinMode(pins.lpwm, OUTPUT);
        pinMode(pins.r_en, OUTPUT);
        pinMode(pins.l_en, OUTPUT);
        digitalWrite(pins.r_en, LOW);
        digitalWrite(pins.l_en, LOW);
        digitalWrite(pins.rpwm, LOW);
        digitalWrite(pins.lpwm, LOW);
    }

    analogWriteRange(PWM_RESOLUTION);
    analogWriteFreq(PWM_FREQUENCY_HZ);
    communicationBegin();
    brakeAll();
    lastValidCommandMs = millis();
    haveValidCommand = false;
}

void loop() {
    const uint32_t now = millis();
    VelocityCommandFrame received{};
    if (communicationPoll(received)) {
        if (velocityIsValid(received)) {
            latestVelocity = received;
            lastValidCommandMs = now;
            haveValidCommand = true;
        } else {
            // Invalid floats or values outside normalized [-1, 1] fail safe.
            ++invalidFrameCount; // intentionally no Serial logging: Serial is binary Blitz
            latestVelocity = {0.0f, 0.0f, 0.0f};
            haveValidCommand = false;
            brakeAll();
        }
    }

    const bool fresh = haveValidCommand
        && static_cast<uint32_t>(now - lastValidCommandMs) <= COMMAND_TIMEOUT_MS;
    if (!fresh) {
        latestVelocity = {0.0f, 0.0f, 0.0f};
        brakeAll();
    } else {
        applyVelocity();
    }
    delay(1);
}
