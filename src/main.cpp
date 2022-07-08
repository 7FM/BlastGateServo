#include <Arduino.h>

#include <FastPin.h>
#include <Servo.h>

#define MIN_SERVO_PULSE 1000
#define MAX_SERVO_PULSE 2000

#define SERVO_MOVE_DURATION 1000
#define SERVO_GATE_OPEN_DEGREE 147
#define SERVO_GATE_CLOSE_DEGREE 25

#define FAN_SLOWDOWN_DURATION 4000

#define SERVO_PIN 2
#define EXTERNAL_SIGNAL_PIN 3
#define RELAY_PIN 4

#if SERVO_PIN == EXTERNAL_SIGNAL_PIN || EXTERNAL_SIGNAL_PIN == RELAY_PIN || SERVO_PIN == RELAY_PIN
#error "Signal, servo & relay pin may not be the same"
#endif

#if SERVO_PIN >= 14 || EXTERNAL_SIGNAL_PIN >= 14 || RELAY_PIN >= 14
#error "We want to minimize power usage, therefore analog pins may not be used for digital purposes!"
#endif

#define _GET_PIN_INTERRUPT_LINE(pin) ((pin) <= 7 ? 2 : (pin) <= 15 ? 0 \
                                                                   : 1)
#define _CAN_PIN_USE_INTX_INTERRUPT_LINE(pin) ((pin) == 2 || (pin) == 3)

#if _CAN_PIN_USE_INTX_INTERRUPT_LINE(EXTERNAL_SIGNAL_PIN)

#if EXTERNAL_SIGNAL_PIN == 2
#define EXTERNAL_SIGNAL_PIN_ISR_VECTOR INT0_vect
#else
#define EXTERNAL_SIGNAL_PIN_ISR_VECTOR INT1_vect
#endif

#else

#if _GET_PIN_INTERRUPT_LINE(EXTERNAL_SIGNAL_PIN) == 2
#define EXTERNAL_SIGNAL_PIN_ISR_VECTOR PCINT2_vect
#elif _GET_PIN_INTERRUPT_LINE(EXTERNAL_SIGNAL_PIN) == 1
#define EXTERNAL_SIGNAL_PIN_ISR_VECTOR PCINT1_vect
#elif _GET_PIN_INTERRUPT_LINE(EXTERNAL_SIGNAL_PIN) == 0
#define EXTERNAL_SIGNAL_PIN_ISR_VECTOR PCINT0_vect
#else
#error "Can not determine interrupt vector for EXTERNAL_SIGNAL_PIN"
#endif

#endif

static constexpr uint8_t calculateExternalInterruptEnableMask() {
    return 0x00 |
           // external interrupt line 0 ist connected to D2
           ((EXTERNAL_SIGNAL_PIN == 2) ? _BV(INT0) : 0x00) |
           // external interrupt line 1 ist connected to D3
           ((EXTERNAL_SIGNAL_PIN == 3) ? _BV(INT1) : 0x00);
}

static constexpr uint8_t calculatePinChangeLine(uint8_t pin) {
    return _CAN_PIN_USE_INTX_INTERRUPT_LINE(pin) ? -1 : _GET_PIN_INTERRUPT_LINE(pin);
}

static constexpr uint8_t calculatePinChangeLineMask(uint8_t pin) {
    return pin <= 7 ? pin : pin <= 15 ? pin - 8
                                      : pin - 16;
}

static constexpr uint8_t calculatePinChangeInterruptEnableMask(uint8_t pinChangeLine) {
    return 0x00 |
           // is signal pin of the encoder on this interrupt line? if set it active!
           ((calculatePinChangeLine(EXTERNAL_SIGNAL_PIN) == pinChangeLine) ? (1 << calculatePinChangeLineMask(EXTERNAL_SIGNAL_PIN)) : 0x00);
}

static constexpr uint8_t calculatePinChangeEnable() {
    return 0x00 |
           // is signal pin on this interrupt line? if so we need to enable the interrupts for this line!
           (calculatePinChangeLine(EXTERNAL_SIGNAL_PIN) == 0 ? _BV(PCIE0) : 0x00) |
           (calculatePinChangeLine(EXTERNAL_SIGNAL_PIN) == 1 ? _BV(PCIE1) : 0x00) |
           (calculatePinChangeLine(EXTERNAL_SIGNAL_PIN) == 2 ? _BV(PCIE2) : 0x00);
}

static inline void setupWakeUpIrq() {
    // Configures wake up interrupt but does not enable it!

    // Configure external interrupts if needed
    if (calculateExternalInterruptEnableMask()) {
        // We want an interrupt on any change!
        // ISCx1    ISCx0   Description
        // 0        0       The low level of INTx generates an interrupt request.
        // 0        1       Any logical change on INTx generates an interrupt request.
        // 1        0       The falling edge of INTx generates an interrupt request.
        // 1        1       The rising edge of INTx generates an interrupt request.
        EICRA = _BV(ISC00) /*| _BV(ISC01)*/ | _BV(ISC10) /*| _BV(ISC11)*/;

        // Enable needed external interrupts
        EIMSK |= calculateExternalInterruptEnableMask();
    }

    // Configure toggle interrupts if needed:
    if (calculatePinChangeEnable()) {
        PCMSK0 = calculatePinChangeInterruptEnableMask(0);
        PCMSK1 = calculatePinChangeInterruptEnableMask(1);
        PCMSK2 = calculatePinChangeInterruptEnableMask(2);

        // PCIE0: Any change on any enabled PCINT7..0 PCINT7..0 pin will cause an interrupt. Pins are enabled individually by the PCMSK0 register.
        // PCIE1: Any change on any enabled PCINT14..8 pin will cause an interrupt. Pins are enabled individually by the PCMSK1 register.
        // PCIE2: Any change on any enabled PCINT23..16 pin will cause an interrupt. Pins are enabled individually by the PCMSK2 register.

        // Enable pin change interrupts
        PCICR |= calculatePinChangeEnable();
    }
}

static inline void enableWakeUpIrq() {
    // enable external interrupts
    if (calculateExternalInterruptEnableMask()) {
        // clear pending interrupt requests
        EIFR = _BV(INTF0) | _BV(INTF1);

        // Enable needed external interrupts
        EIMSK |= calculateExternalInterruptEnableMask();
    }

    // enable toggle interrupts
    if (calculatePinChangeEnable()) {
        // clear pending interrupt requests
        PCIFR = _BV(PCIF0) | _BV(PCIF1) | _BV(PCIF2);

        // Enable pin change interrupts
        PCICR |= calculatePinChangeEnable();
    }
}

static inline void disableWakeUpIrq() {
    // disable needed external interrupts
    if (calculateExternalInterruptEnableMask()) {
        EIMSK &= ~calculateExternalInterruptEnableMask();
    }

    // disable pin change interrupts
    if (calculatePinChangeEnable()) {
        PCICR &= ~calculatePinChangeEnable();
    }
}

static void awaitServoDone() {
    // Wait for the servo beeing disabled, aka output compare interrupt gets disabled
    while (TIMSK1 &= _BV(OCIE1A)) {
    }
}

static void enterPowerDownMode() {
    // Power down mode has only SM1 set but we need to enable sleep to really enter sleep on the sleep instruction
    SMCR = SLEEP_MODE_PWR_DOWN /*SM1*/ | SE;
    enableWakeUpIrq();
    // Enter sleep mode
    __asm__ __volatile__(
        "sleep\n"
        :
        :
        :);
    // Clear sleep enable to avoid unwanted sleeps!
    SMCR &= ~SE;
}

static void setupPins() {
    // Signal pin: input & pull down!
    FastPin<EXTERNAL_SIGNAL_PIN>::setInput();
    FastPin<EXTERNAL_SIGNAL_PIN>::lo();

    // Servo pin output & low
    FastPin<SERVO_PIN>::setOutput();
    FastPin<SERVO_PIN>::lo();

    // Relay pin output & low
    FastPin<RELAY_PIN>::setOutput();
    FastPin<RELAY_PIN>::lo();
}

static void disableDigitalOnAnalogPins() {
    // disable digitial input buffer on analog pins, as not used and saves energy
    DIDR0 = ADC0D | ADC1D | ADC2D | ADC3D | ADC4D | ADC5D;
    DIDR1 = AIN0D | AIN1D;
}

ISR(EXTERNAL_SIGNAL_PIN_ISR_VECTOR) {
    // Executes only at wake up + ensure that by disabling it after wake up!
    disableWakeUpIrq();
}

// Our servo control instance
static Servo servo;

void setup() {
    setupPins();
    disableDigitalOnAnalogPins();
    setupWakeUpIrq();

    // setup servo
    servo.attach(SERVO_PIN, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
}

void loop() {
    bool requestOpen = FastPin<EXTERNAL_SIGNAL_PIN>::digitalRead();

    // Check the requested command!
    if (requestOpen) {
        // Open blast gate
        servo.write(SERVO_GATE_OPEN_DEGREE, SERVO_MOVE_DURATION);
        // Wait until its open
        awaitServoDone();
        // Turn fan on
        FastPin<RELAY_PIN>::hi();
    } else {
        // Turn fan off
        FastPin<RELAY_PIN>::lo();
        // wait until fan slowed down!
        _delay_ms(FAN_SLOWDOWN_DURATION);
        // Close the blast gate
        servo.write(SERVO_GATE_CLOSE_DEGREE, SERVO_MOVE_DURATION);
        // Wait until close is executed
        awaitServoDone();
    }

    // Check if we missed a level change -> dont go to sleep yet but apply new request
    // else enter power down mode
    if (requestOpen == FastPin<EXTERNAL_SIGNAL_PIN>::digitalRead()) {
        // Sleep until next change to save energy!
        enterPowerDownMode();
    }
}
