#ifndef __BUZZER__
#define __BUZZER__

/**
 * @brief Buzzer control class for audio feedback
 * Manages PWM-based sound generation for status indicators
 */
class Buzzer {
public:
    /**
     * @brief Construct a new Buzzer object
     * @param pin GPIO pin number connected to the buzzer
     */
    explicit Buzzer(int pin) : pin(pin) {}

    /**
     * @brief Standard notification beep
     * Medium frequency (500Hz), medium duration (500ms)
     */
    void beep() {
        generateTone(500, 500);
    }

    /**
     * @brief Error indication beep
     * Low frequency (100Hz), long duration (1000ms)
     */
    void beepError() {
        generateTone(100, 1000);
    }

    /**
     * @brief Short acknowledgment beep
     * Very low frequency (50Hz), short duration (100ms)
     */
    void beepShort() {
        generateTone(50, 100);
    }

private:
    // PWM configuration
    static constexpr uint8_t PWM_RESOLUTION = 8;
    static constexpr uint8_t PWM_CHANNEL = 15;
    static constexpr uint8_t DUTY_CYCLE_ON = (1 << (PWM_RESOLUTION - 1));  // 50% duty cycle
    static constexpr uint8_t DUTY_CYCLE_OFF = 0;                           // 0% duty cycle

    int pin;  // GPIO pin number

    /**
     * @brief Generate a tone using PWM
     * @param frequency Tone frequency in Hz
     * @param duration Duration in milliseconds
     */
    void generateTone(int frequency, int duration) {
        // Configure PWM
        ledcSetup(PWM_CHANNEL, frequency, PWM_RESOLUTION);
        ledcAttachPin(pin, PWM_CHANNEL);

        // Generate sound
        ledcWrite(PWM_CHANNEL, DUTY_CYCLE_ON);
        delay(duration);
        ledcWrite(PWM_CHANNEL, DUTY_CYCLE_OFF);

        // Cleanup
        ledcDetachPin((gpio_num_t)pin);
    }
};

#endif // __BUZZER__
