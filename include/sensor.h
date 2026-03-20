#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

class Sensor{

    public:
        Sensor();
        void setSensorPins(uint16_t emitter_pin, uint16_t reciever_pin);
        void initSensor();
        void updateDistance();
        float getDistance();
        void setCalibration(float minVal, float maxVal);
        float getNormalizedDistance();

    private:
        uint16_t emitter_pin, reciever_pin;
        float lastDistance;
        float offTime, resetTime;
        int readCount = 5;
        int calibMin, calibMax;
};
#endif