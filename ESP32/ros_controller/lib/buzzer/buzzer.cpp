#include "buzzer.h"

Buzzer::Buzzer(uint8_t pin, uint8_t channel)
    : _pin(pin), _channel(channel), _stopTime(0), _playing(false) {
    // Configure PWM channel with 8-bit resolution
    ledcSetup(_channel, 1000, PWM_RESOLUTION);
    // Attach the channel to the GPIO pin
    ledcAttachPin(_pin, _channel);
    // Make sure buzzer is off initially
    toneOff();
}


void Buzzer::tone(uint32_t frequency, uint32_t duration) {
    // Blocking version - play tone and wait
    toneOn(frequency);
    delay(duration);
    toneOff();
}

void Buzzer::toneOn(uint32_t frequency) {
    if (frequency > 0) {
        // Set the frequency and start the tone with 50% duty cycle
        ledcWriteTone(_channel, frequency);
        ledcWrite(_channel, 128);  // 50% duty cycle (128/255)
        _playing = true;
    } else {
        toneOff();
    }
}

void Buzzer::toneOff() {
    // Stop the tone by setting duty cycle to 0
    ledcWrite(_channel, 0);
    _playing = false;
    _stopTime = 0;
}

void Buzzer::playTone(uint32_t frequency, uint32_t duration) {
    // Non-blocking version - start tone and set stop time
    toneOn(frequency);
    _stopTime = millis() + duration;
}

void Buzzer::update() {
    // Check if it's time to stop the tone
    if (_playing && _stopTime > 0 && millis() >= _stopTime) {
        toneOff();
    }
}

bool Buzzer::isPlaying() const {
    return _playing;
}

void Buzzer::welcomeTune() {
    // Play a cheerful startup melody
    tone(523, 150);   // C5
    delay(50);
    tone(659, 150);   // E5
    delay(50);
    tone(784, 150);   // G5
    delay(50);
    tone(1047, 300);  // C6
    delay(100);
}

void Buzzer::errorTune() {
    // Play an error/alert sound
    tone(800, 200);   // High
    delay(50);
    tone(400, 200);   // Low
    delay(50);
    tone(800, 200);   // High
    delay(50);
    tone(400, 300);   // Low (longer)
}

void Buzzer::byeTune() {
    // Play a goodbye melody (descending)
    tone(1047, 150);  // C6
    delay(50);
    tone(784, 150);   // G5
    delay(50);
    tone(659, 150);   // E5
    delay(50);
    tone(523, 300);   // C5
    delay(100);
}
