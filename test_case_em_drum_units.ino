/**
 * Test-Case - EM Drum Units
 *
 * Version 1.0 (01-12-2021)
 **/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/** Define Arduino pins and PCA9685 pins **/
#define PIN_LED_HEARTBEAT 9

#define PIN_MATRIX_IN_0 A0
#define PIN_MATRIX_IN_1 A1
#define PIN_MATRIX_IN_2 A2
#define PIN_MATRIX_IN_3 A3

#define PIN_MATRIX_OUT_0 4
#define PIN_MATRIX_OUT_1 5
#define PIN_MATRIX_OUT_2 6
#define PIN_MATRIX_OUT_3 7

#define PIN_PCA9685_OE 8

//  0 - Solenoid: Drum Unit Ones
//  1 - Solenoid: Drum Unit Tens
//  2 - Solenoid: Drum Unit Hundreds
//  3 - Solenoid: Drum Unit Thousands
//  4 - LED: Unused
//  5 - LED: Unused
//  6 - LED: Unused
//  7 - LED: Unused
#define PIN_PCA9685_DRUM_UNIT_1 0
#define PIN_PCA9685_DRUM_UNIT_10 1
#define PIN_PCA9685_DRUM_UNIT_100 2
#define PIN_PCA9685_DRUM_UNIT_1000 3

//  8 - LED: Thousands Button
//  9 - LED: Hundreds Button
// 10 - LED: Tens Button
// 11 - LED: Ones Button
// 12 - LED: Reset Button
// 13 - LED: Unused
// 14 - LED: Unused
// 15 - LED: Unused
#define PIN_PCA9685_LED_BUTTON_1000 8
#define PIN_PCA9685_LED_BUTTON_100 9
#define PIN_PCA9685_LED_BUTTON_10 10
#define PIN_PCA9685_LED_BUTTON_1 11
#define PIN_PCA9685_LED_BUTTON_RESET 12

/** Type definitions **/
typedef void (*SwitchCallback)();

struct SwitchState {
    uint32_t first_change;
    bool is_closed;
    SwitchCallback on_switch_close;
    SwitchCallback on_switch_open;
};

enum LedAnimationState {
    Running = 0,
    Pauzed = 1,
    FadeIn = 2,
    FadeOut = 3
};

enum LedAnimationType {
    Static = 0,
    Fade = 1
};

struct LedAnimationStep {
    LedAnimationType type;
    uint32_t duration;
    uint16_t pwm_value;
};

struct LedAnimation {
    LedAnimationStep* steps;
    uint32_t last_update;
    uint8_t current_step;
    uint16_t current_pwm_value;
    uint16_t backup_pwm_value;
    uint8_t number_of_steps;
    uint8_t pin;
    LedAnimationState state;
};

struct DrumUnit {
    uint8_t pin;
    uint32_t timer;
    uint8_t position;
};

/** Global variables **/

// The PCA9685 driver object
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// Whether the initial reset should still be performed
bool state_should_reset = true;

// To keep track whether we're resetting for the first time or not
bool state_is_initial_reset = true;

// Starts out on false, as it has to wait for the inital reset and it is false during regular resets
bool state_is_drum_unit_allowed = false;

// Is false when the reset button can not be used
bool state_is_reset_allowed = false;

// Track whether another drum unit needs to be triggered to simulate a carry
bool state_has_carry_for_10 = false;
bool state_has_carry_for_100 = false;
bool state_has_carry_for_1000 = false;

/** Configuration **/
#define MATRIX_DEBOUNCE_MS 20
#define MATRIX_HEIGHT 4
#define MATRIX_WIDTH 4

#define DRUM_UNIT_MS 50
#define DRUM_UNIT_RESET_STEP_MS 200

#define LED_FADE_SPEED_MS 200
#define LED_MAX_VALUE 4096
#define LED_HALF_VALUE 2048
#define LED_QUARTER_VALUE 1024

#define INITIAL_RESET_AFTER_MS 250

/** Heartbeat LED **/
uint8_t heartbeat_led_index = 0;
uint32_t heartbeat_led_sequence[4] = { 200, 125, 200, 600 };
uint32_t heartbeat_led_update = 0;
bool heartbeat_led_state = true;

void init_heartbeat_led() {
    // Set up the heartbeat LED pin
    pinMode(PIN_LED_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_LED_HEARTBEAT, heartbeat_led_state);
}

void update_heartbeat_led() {
    if (millis() - heartbeat_led_update < heartbeat_led_sequence[heartbeat_led_index]) {
        // Nothing to update yet
        return;
    }

    // Increase the index and make sure it does not go out of bounds
    heartbeat_led_index++;
    if (heartbeat_led_index == 4) {
        heartbeat_led_index = 0;
    }

    // Mark now as the last update
    heartbeat_led_update = millis();

    // Update the pin state and flip the internal state
    digitalWrite(PIN_LED_HEARTBEAT, heartbeat_led_state);
    heartbeat_led_state = !heartbeat_led_state;
}

/** Switch Matrix **/
uint8_t matrix_input_pins[MATRIX_HEIGHT] = {
    PIN_MATRIX_IN_0,
    PIN_MATRIX_IN_1,
    PIN_MATRIX_IN_2,
    PIN_MATRIX_IN_3
};
uint8_t matrix_output_pins[MATRIX_WIDTH] = {
    PIN_MATRIX_OUT_0,
    PIN_MATRIX_OUT_1,
    PIN_MATRIX_OUT_2,
    PIN_MATRIX_OUT_3
};

uint8_t matrix_col = 0;
uint8_t matrix_row = 0;

// Scratchpad pointer for loops iterating over switch state structs
SwitchState* switch_state = NULL;

SwitchState matrix[MATRIX_WIDTH][MATRIX_HEIGHT] = {
    {
        // 0x0 - NO - Front button 1: Green / Thousands Button
        {0, false, on_1000_button_down, NULL},
        // 0x1 - NO - Front button 2: Green / Hundreds Button
        {0, false, on_100_button_down, NULL},
        // 0x2 - NO - Front button 3: Green / Tens Button
        {0, false, on_10_button_down, NULL},
        // 0x3 - NO - Front Button 4: Green / Ones Button
        {0, false, on_1_button_down, NULL}
    },
    {
        // 1x0 - NO - Front button 5: Red / Reset Button
        {0, false, on_reset_button_down, NULL},
        // 1x1 - NO - Unused
        {0, false, NULL, NULL},
        // 1x2 - NO - Unused
        {0, false, NULL, NULL},
        // 1x3 - NO - Unused
        {0, false, NULL, NULL}
    },
    {
        // 2x0 - NO - Thousands Carry Switch
        {0, false, NULL, NULL},
        // 2x1 - NO - Hundreds Carry Switch
        {0, false, on_100_carry_closed, on_100_carry_opened},
        // 2x2 - NO - Tens Carry Switch
        {0, false, on_10_carry_closed, on_10_carry_opened},
        // 2x3 - NO - Ones Carry Switch
        {0, false, on_1_carry_closed, on_1_carry_opened}
    },
    {
        // 3x0 - NC - Thousands Zero Switch
        {0, true, NULL, NULL},
        // 3x1 - NC - Hundreds Zero Switch
        {0, true, NULL, NULL},
        // 3x2 - NC - Tens Zero Switch
        {0, true, NULL, NULL},
        // 3x3 - NC - Ones Zero Switch
        {0, true, NULL, NULL}
    }
};

void init_matrix() {
    // Set pin mode for output pins
    for (matrix_col = 0; matrix_col < MATRIX_WIDTH; matrix_col++) {
        pinMode(matrix_output_pins[matrix_col], OUTPUT);
    }

    // Set pin mode for input pins
    for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
        pinMode(matrix_input_pins[matrix_row], INPUT);
    }
}

void update_matrix() {
    for (matrix_col = 0; matrix_col < MATRIX_WIDTH; matrix_col++) {
        // Set output pins
        for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
            digitalWrite(matrix_output_pins[matrix_row], matrix_col != matrix_row);
        }

        // Scan input pins
        for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
            switch_state = &matrix[matrix_row][matrix_col];

            // Check if switch state has been changed
            if (digitalRead(matrix_input_pins[matrix_row]) == (switch_state->is_closed ? HIGH : LOW)) {
                if (!switch_state->first_change) {
                    // Mark the first time this state change has been seen
                    switch_state->first_change = millis();
                }
                else {
                    // Check if switch is in its new state for long enough
                    if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                        if (switch_state->is_closed) {
                            // Call the on switch open callback
                            if (switch_state->on_switch_open != NULL) {
                                switch_state->on_switch_open();
                            }
                        }
                        else {
                            // Call the on switch close callback
                            if (switch_state->on_switch_close != NULL) {
                                switch_state->on_switch_close();
                            }
                        }

                        // Update internal state
                        switch_state->is_closed = !switch_state->is_closed;
                        switch_state->first_change = 0;
                    }
                }
            }
            else if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                // Reset internal state if switch went back to its previous state within the debounce time
                switch_state->first_change = 0;
            }
        }
    }
}

/** PCA9685 **/
void init_pca9685() {
    // Constructor initializes a PCA9685 device at default address 0x40
    pca9685.begin();
    // Set maximum PWM frequency in Hz
    pca9685.setPWMFreq(1600);
    // Set output to push/pull (totempole)
    pca9685.setOutputMode(true);

    // Set Output Enable (OE) pin low
    pinMode(PIN_PCA9685_OE, OUTPUT);
    digitalWrite(PIN_PCA9685_OE, LOW);
}

/** Solenoid Control **/
DrumUnit drum_units[4] = {
    { PIN_PCA9685_DRUM_UNIT_1, 0, 0 },
    { PIN_PCA9685_DRUM_UNIT_10, 0, 0 },
    { PIN_PCA9685_DRUM_UNIT_100, 0, 0 },
    { PIN_PCA9685_DRUM_UNIT_1000, 0, 0 }
};

// Scratchpad pointer for loops iterating over drum units
DrumUnit* drum_unit = NULL;

// Timer used to track time between drum unit reset pulses
uint32_t drum_unit_reset_timer = 0;

// Counter to keep track of which drum unit to look at next while resetting
uint8_t drum_unit_next_to_look_at = 3;

void fire_drum_unit(uint8_t index) {
    drum_unit = &drum_units[index];

    // Set timer and switch on MOSFET
    drum_unit->timer = millis();
    pca9685.setPin(drum_unit->pin, 4096, false);

    // Update internally tracked position
    drum_unit->position++;
    if (drum_unit->position > 9) {
        drum_unit->position = 0;
    }
}

void update_drum_units() {
    // Update drum unit solenoids
    for (uint8_t i = 0; i < 4; i++) {
        drum_unit = &drum_units[i];

        if (drum_unit->timer) {
            if (millis() - drum_unit->timer > DRUM_UNIT_MS) {
                // Reset timer and switch off MOSFET
                pca9685.setPin(drum_unit->pin, 0, false);
                drum_unit->timer = 0;
            }
        }
    }

    // Update drum unit reset sequence
    if (drum_unit_reset_timer) {
        if (millis() - drum_unit_reset_timer > DRUM_UNIT_RESET_STEP_MS) {
            // Try to actually reset a drum unit (one step)
            bool did_reset = drum_unit_reset_next();

            if (did_reset) {
                // It did reset, set the timer for the next unit
                drum_unit_reset_timer = millis();
            }
            else {
                // It did not reset, reset the timer and update the state
                drum_unit_reset_timer = 0;
                state_is_drum_unit_allowed = true;

                if (state_is_initial_reset) {
                    state_is_initial_reset = false;

                    // Set the internally tracked position to 0, as it is now in a known state
                    drum_units[0].position = 0;
                    drum_units[1].position = 0;
                    drum_units[2].position = 0;
                    drum_units[3].position = 0;
                }

                update_state();
            }
        }
    }
}

bool drum_unit_reset_next() {
    for (uint8_t i = 0; i < 4; i++) {
        // Increment the next unit to look at and make sure it does not go out of bounds
        drum_unit_next_to_look_at++;
        if (drum_unit_next_to_look_at == 4) {
            drum_unit_next_to_look_at = 0;
        }

        // Fire the drum unit if its 0-position switch is closed
        if (matrix[3][drum_unit_next_to_look_at].is_closed) {
            fire_drum_unit(drum_unit_next_to_look_at);
            return true;
        }
    }

    // Reset next unit to look at counter
    drum_unit_next_to_look_at = 3;
    return false;
}

void reset_drum_units() {
    // Start the reset sequence by setting the timer
    drum_unit_reset_timer = millis();

    // Update the state
    state_is_drum_unit_allowed = false;
}

/** LED animations **/

// Thousands button animation
LedAnimationStep led_1000_animation_sequence[6] = {
    { LedAnimationType::Static, 125, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2775, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 500, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2400, LED_QUARTER_VALUE }
};

LedAnimation led_1000_animation = {
    led_1000_animation_sequence,
    1,
    0,
    0,
    LED_HALF_VALUE,
    6,
    PIN_PCA9685_LED_BUTTON_1000,
    LedAnimationState::Running
};

// Hundreds button animation
LedAnimationStep led_100_animation_sequence[6] = {
    { LedAnimationType::Static, 250, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2650, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 375, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2525, LED_QUARTER_VALUE }
};

LedAnimation led_100_animation = {
    led_100_animation_sequence,
    1,
    0,
    0,
    LED_HALF_VALUE,
    6,
    PIN_PCA9685_LED_BUTTON_100,
    LedAnimationState::Running
};

// Tens button animation
LedAnimationStep led_10_animation_sequence[6] = {
    { LedAnimationType::Static, 375, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2525, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 250, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2650, LED_QUARTER_VALUE }
};

LedAnimation led_10_animation = {
    led_10_animation_sequence,
    1,
    0,
    0,
    LED_HALF_VALUE,
    6,
    PIN_PCA9685_LED_BUTTON_10,
    LedAnimationState::Running
};

// Ones button animation
LedAnimationStep led_1_animation_sequence[6] = {
    { LedAnimationType::Static, 500, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2400, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 125, LED_QUARTER_VALUE },
    { LedAnimationType::Static, 100, LED_MAX_VALUE },
    { LedAnimationType::Static, 2775, LED_QUARTER_VALUE }
};

LedAnimation led_1_animation = {
    led_1_animation_sequence,
    1,
    0,
    0,
    LED_HALF_VALUE,
    6,
    PIN_PCA9685_LED_BUTTON_1,
    LedAnimationState::Running
};

// Reset button animation
LedAnimationStep led_reset_animation_sequence[1] = {
    { LedAnimationType::Static, 1000, LED_HALF_VALUE }
};

LedAnimation led_reset_animation = {
    led_reset_animation_sequence,
    1,
    0,
    0,
    LED_HALF_VALUE,
    1,
    PIN_PCA9685_LED_BUTTON_RESET,
    LedAnimationState::Pauzed
};

#define LED_ANIMATION_NUMBER_OF 5
LedAnimation animations[LED_ANIMATION_NUMBER_OF] = {
    led_1000_animation,
    led_100_animation,
    led_10_animation,
    led_1_animation,
    led_reset_animation
};

// Scratchpad pointers for loops iterating over animations and animation steps
LedAnimation* animation = NULL;
LedAnimationStep* step = NULL;

// Warning: Hairy code ahead
void update_leds() {
    for (uint8_t i = 0; i < LED_ANIMATION_NUMBER_OF; i++) {
        animation = &animations[i];

        if (animation->state == LedAnimationState::Pauzed) {
            // Animation is pauzed, don't do anything
            continue;
        }

        if (animation->state == LedAnimationState::FadeOut) {
            // Alternate flow, fading out is not part of the normal animation flow
            if (millis() - animation->last_update > LED_FADE_SPEED_MS) {
                // The fade out is done, set the current pwm value to 0
                animation->current_pwm_value = 0;
                pca9685.setPin(animation->pin, animation->current_pwm_value, true);
                // Update animation state
                animation->state = LedAnimationState::Pauzed;
                continue;
            }

            // Compute intermediary value of the fade out
            uint32_t time_in = millis() - animation->last_update;
            animation->current_pwm_value = map(time_in, 0, LED_FADE_SPEED_MS, animation->backup_pwm_value, 0);
            pca9685.setPin(animation->pin, animation->current_pwm_value, true);
            continue;
        }

        if (animation->state == LedAnimationState::FadeIn) {
            // Alternate flow, fading in is not part of the normal animation flow
            step = &animation->steps[0];

            if (millis() - animation->last_update > LED_FADE_SPEED_MS) {
                // The fade in is done, set the current pwm value to the first step's pwm value
                animation->current_pwm_value = step->pwm_value;
                pca9685.setPin(animation->pin, animation->current_pwm_value, true);
                // Update animation state. Let the animation resetart from the first step
                animation->state = LedAnimationState::Running;
                animation->last_update = millis();
                animation->current_step = 0;
                continue;
            }

            // Compute intermediary value of the fade in
            uint32_t time_in = millis() - animation->last_update;
            animation->current_pwm_value = map(time_in, 0, LED_FADE_SPEED_MS, 0, step->pwm_value);
            pca9685.setPin(animation->pin, animation->current_pwm_value, true);
            continue;
        }

        step = &animation->steps[animation->current_step];

        if (step->type == LedAnimationType::Fade) {
            // Compute intermediary value of the fade step
            uint16_t previous_pwm_value = animation->steps[animation->current_step == 0 ? animation->number_of_steps - 1 : animation->current_step - 1].pwm_value;
            uint32_t time_in = millis() - animation->last_update;
            animation->current_pwm_value = map(time_in, 0, step->duration, previous_pwm_value, step->pwm_value);
            pca9685.setPin(animation->pin, animation->current_pwm_value, true);
        }

        if (millis() - animation->last_update > step->duration) {
            // Move on to the next animation step if the current step is done
            animation->last_update = millis();
            animation->current_step += 1;
            if (animation->current_step == animation->number_of_steps) {
                animation->current_step = 0;
            }

            step = &animation->steps[animation->current_step];
            if (step->type == LedAnimationType::Static) {
                // If the step is a static pwm value, set it
                animation->current_pwm_value = step->pwm_value;
                pca9685.setPin(animation->pin, step->pwm_value, true);
            }
        }
    }
}

void stop_animation(uint8_t animation_index) {
    animation = &animations[animation_index];

    if (animation->state == LedAnimationState::Pauzed) {
        // Don't do anything if the animation is already pauzed
        return;
    }

    // Update animation state
    animation->state = LedAnimationState::FadeOut;
    animation->last_update = millis();
    // Set the backup pwm field. This value is needed when computing the new current
    // value while fading out. The pwm value of the current step can not be used as
    // that step might not yet be completed.
    animation->backup_pwm_value = animation->current_pwm_value;
}

void resume_animation(uint8_t animation_index) {
    animation = &animations[animation_index];

    if (animation->state == LedAnimationState::Running) {
        // Don't do anything if the animation is already running
        return;
    }

    // Update animation state
    animation->state = LedAnimationState::FadeIn;
    animation->last_update = millis();
}

/** Serial **/
void init_serial() {
    // Open the serial connection
    Serial.begin(9600);
}

void update_serial() {
    // Check for available characters on the serial port.
    if (Serial.available() > 0) {
        // If the sent character is a "c", print the credits "window"
        if (Serial.read() == 'c') {
            Serial.println("------ Test-Case - EM Drum Units  ------\n        Version 1.0 (01-12-2021)\n       Made in The Netherlands by\n    Cor Gravekamp & Thomas Gravekamp\n                 for the\n          Dutch Pinball Museum\n----------------------------------------");
        }
    }
}

/** Button actions **/
void on_1000_button_down() {
    if (!state_is_drum_unit_allowed || drum_units[0].timer) {
        // Prevent firing drum unit while resetting or while firing
        return;
    }

    fire_drum_unit(0);

    update_state();
}

void on_100_button_down() {
    if (!state_is_drum_unit_allowed || drum_units[1].timer) {
        // Prevent firing drum unit while resetting or while firing
        return;
    }

    fire_drum_unit(1);

    if (state_has_carry_for_1000) {
        fire_drum_unit(0);
        state_has_carry_for_1000 = false;
    }

    update_state();
}

void on_10_button_down() {
    if (!state_is_drum_unit_allowed || drum_units[2].timer) {
        // Prevent firing drum unit while resetting or while firing
        return;
    }

    fire_drum_unit(2);

    if (state_has_carry_for_100) {
        fire_drum_unit(1);
        state_has_carry_for_100 = false;

        if (state_has_carry_for_1000) {
            fire_drum_unit(0);
            state_has_carry_for_1000 = false;
        }
    }

    update_state();
}

void on_1_button_down() {
    if (!state_is_drum_unit_allowed || drum_units[3].timer) {
        // Prevent firing drum unit while resetting or while firing
        return;
    }

    fire_drum_unit(3);

    if (state_has_carry_for_10) {
        fire_drum_unit(2);
        state_has_carry_for_10 = false;

        if (state_has_carry_for_100) {
            fire_drum_unit(1);
            state_has_carry_for_100 = false;

            if (state_has_carry_for_1000) {
                fire_drum_unit(0);
                state_has_carry_for_1000 = false;
            }
        }
    }

    update_state();
}

void on_reset_button_down() {
    reset_drum_units();

    update_state();
}

void on_100_carry_closed() {
    state_has_carry_for_1000 = true;
}

void on_100_carry_opened() {
    state_has_carry_for_1000 = false;
}

void on_10_carry_closed() {
    state_has_carry_for_100 = true;
}

void on_10_carry_opened() {
    state_has_carry_for_100 = false;
}

void on_1_carry_closed() {
    state_has_carry_for_10 = true;
}

void on_1_carry_opened() {
    state_has_carry_for_10 = false;
}

void update_state() {
    // If all drum units are at position 0, do not allow resetting
    // Only update the animation when the new state differs from the old state
    if ((drum_units[0].position == 0 &&
        drum_units[1].position == 0 &&
        drum_units[2].position == 0 &&
        drum_units[3].position == 0) != state_is_reset_allowed) {

        // Update state
        state_is_reset_allowed = !state_is_reset_allowed;

        // Update reset button LED animation
        if (state_is_reset_allowed) {
            stop_animation(4);
        }
        else {
            resume_animation(4);
        }
    }

    // Drum unit buttons can not be pressed while resetting, update their LED animations accordingly
    if (!state_is_drum_unit_allowed && !(
        drum_units[0].position == 0 &&
        drum_units[1].position == 0 &&
        drum_units[2].position == 0 &&
        drum_units[3].position == 0)) {
        stop_animation(0);
        stop_animation(1);
        stop_animation(2);
        stop_animation(3);
    }
    else {
        resume_animation(0);
        resume_animation(1);
        resume_animation(2);
        resume_animation(3);
    }
}

void setup() {
    // Initialize heartbeat LED
    init_heartbeat_led();

    // Initialize switch matrix
    init_matrix();

    // Initialize PCA9685
    init_pca9685();

    // Initialize serial
    init_serial();
}

void loop() {
    // Update heartbeat LED
    update_heartbeat_led();

    // Update switch matrix
    update_matrix();

    // Update drum units
    update_drum_units();

    // Update LEDs
    update_leds();

    // Update serial
    update_serial();

    // Reset the drum units 250ms after startup to move them to a known position
    if (state_should_reset && millis() > INITIAL_RESET_AFTER_MS) {
        reset_drum_units();
        state_should_reset = false;
    }
}
