#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

class Buzzer {
public:
    /**
     * @brief Construct a new Buzzer object
     * 
     * @param pin GPIO pin connected to the buzzer
     * @param channel PWM channel to use (0-15)
     */
    Buzzer(uint8_t pin, uint8_t channel = 0);

    /**
     * @brief Play a tone at specified frequency for specified duration (blocking)
     * 
     * @param frequency Frequency in Hz
     * @param duration Duration in milliseconds
     */
    void tone(uint32_t frequency, uint32_t duration);

    /**
     * @brief Start playing a tone at specified frequency (non-blocking)
     * 
     * @param frequency Frequency in Hz
     */
    void toneOn(uint32_t frequency);

    /**
     * @brief Stop playing the current tone
     */
    void toneOff();

    /**
     * @brief Play a tone for a duration (non-blocking version)
     * Call update() in loop to handle timing
     * 
     * @param frequency Frequency in Hz
     * @param duration Duration in milliseconds
     */
    void playTone(uint32_t frequency, uint32_t duration);

    /**
     * @brief Update function - call this in loop() for non-blocking operation
     */
    void update();

    /**
     * @brief Check if buzzer is currently playing
     * 
     * @return true if playing
     * @return false if not playing
     */
    bool isPlaying() const;

    /**
     * @brief Play a welcome tune (startup melody)
     */
    void welcomeTune();

    /**
     * @brief Play an error tune (alert sound)
     */
    void errorTune();

    /**
     * @brief Play a warning tune (caution sound)
     */
    void warningTune();

    /**
     * @brief Play a goodbye tune (shutdown melody)
     */
    void byeTune();

private:
    uint8_t _pin;
    uint8_t _channel;
    uint32_t _stopTime;
    bool _playing;
    static constexpr uint32_t PWM_RESOLUTION = 8;  // 8-bit resolution
};

#endif // BUZZER_H
