#include <Arduino.h>
#include <sensor.h>
#include <stdint.h>

Sensor::Sensor(){};

void Sensor::setSensorPins(uint16_t ep, uint16_t rp){
    emitter_pin = ep;
    reciever_pin = rp;
}

void Sensor::setCalibration(float minVal, float maxVal){
    calibMin = minVal;
    calibMax = maxVal;
}

float Sensor::getNormalizedDistance(){
    float raw = getDistance();
    return (raw - calibMin) / (calibMax - calibMin);
}

void Sensor::initSensor(){
    pinMode(emitter_pin, OUTPUT);
    pinMode(reciever_pin, INPUT);
    digitalWrite(emitter_pin, LOW);
    lastDistance = 0;
}

float Sensor::getDistance(){
    updateDistance();
    return lastDistance;
}

void Sensor::updateDistance(){
    //if(offTime >= resetTime){
        int initReading = analogRead(reciever_pin);
        //Serial.println(initReading);

        digitalWrite(emitter_pin, HIGH);
        delayMicroseconds(200);

        int readAvg = 0;
        for(int i = 0; i < readCount; i++){
            readAvg += analogRead(reciever_pin);
            //Serial.println(readAvg);
        }

        digitalWrite(emitter_pin, LOW);
        
        //Serial.printf("%f %f %f\n",  (float) readAvg, (float) readCount, (float) initReading);
        float rawReading = (float) readAvg / (float) readCount - (float) initReading;

        lastDistance = rawReading;
        //lastDistance = (-3.539e-6 * pow(rawReading, 3)) + (2.528e-3 * pow(rawReading, 2)) - (7.085e-1 * rawReading) + (9.664e1);

    //}
}