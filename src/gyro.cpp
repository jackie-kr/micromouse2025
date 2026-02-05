#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

void setup() {
    Serial.begin(115200);
    myIMU.begin();
    delay(1000);
    int8_t temp=myIMU.getTemp();
    myIMU.setExtCrystalUse(true);
}
void loop(){
    
    imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    Serial.print(acc.x());
    Serial.print("     ");
    Serial.print(acc.y());
    Serial.print("     ");
    Serial.print(acc.z());
}
