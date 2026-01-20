#include <Arduino.h>
#include <Stepper.h>

Stepper stepper = Stepper(1);
const byte numChars;
char receivedChars[numChars];
boolean newData = false;
int datNumber = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("Arduino is ready")
}

void loop() {
    if (Serial.available()>0) {
        distance = Serial.read();
        if (typeid(distance)=='f') {
            Serial.print(distance);
            up();
        }
    }
}

void up() {
    stepper.step(1)
    stepper.delay(10)
}

void down() {
    stepper.step(-1)
    stepper.delay(10)
}