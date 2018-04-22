#include <Arduino.h>


class DeadMan{
    public:
    DeadMan(uint8_t pulse_pin, uint32_t watch_time){
        pin = pulse_pin;
        time_limit = watch_time;
        pinMode(pin,OUTPUT);
        pulse_state = LOW;
        watchdog_timer = millis();
        pulse_timer = millis();
    }

    void pet(){
        watchdog_timer = millis();
    }

    void check(){
        if (millis() > (watchdog_timer + time_limit))
        { //if skybass is still alive
            if (millis() > pulse_timer + 200)
            { //if its been a while...
                pulse_state = !pulse_state;
                digitalWrite(pin, pulse_state);
            }
        }

    }

    private:
    uint8_t pin;
    uint32_t time_limit;
    uint32_t watchdog_timer;
    uint32_t pulse_timer;
    bool pulse_state;
};