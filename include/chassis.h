#pragma once
#include <motor.h>
#include <MiniPID.h>
#include <elapsedMillis.h>
//#include "MPU9250.h"
#include <Adafruit_BNO055.h>

class Chassis{

    private:
        float wheelDiameter;
        float wheelTrack;
        float encRatio;
        struct position {double x; double y; double rotation;};
        position currentPos;
        Motor *backRightMotor;
        Motor *frontRightMotor;
        Motor *backLeftMotor;
        Motor *frontLeftMotor;
        double pointDistance(position pos1, position pos2);
        double pointAngle(position pos1, position pos2);
        int lastLeftEnc;
        int lastRightEnc;
        MiniPID *distancePID;
        MiniPID *anglePID;
        MiniPID *turnPID;
        Adafruit_BNO055 *bno;
        float distanceError;
        float angleError;
        elapsedMicros turnTargetTime;
        double getLeftEnc();
        double getRightEnc();

    public:
        Chassis();
        void setMotors(Motor *brm, Motor *blm, Motor *frm, Motor *flm);
        void setChassisAttr(float ws, float er, float wt);
        void setPID(MiniPID *dPID, MiniPID *aPID, MiniPID *tPID);
        void setIMU(Adafruit_BNO055* imu);
        void updatePosition();
        void moveToPostion(double x, double y);
        void turnToOrientation(double theta);
        void driveVector(double velocity, double theta);
        void setError(float dError, float aError);
        void printPosition();
        bool turnIsSettled();
        bool gyroTurnIsSettled();
        void gyroTurnOrientation(double theta);
        void turnOrientation(double theta);
        void moveForwardTile();
        void turnLeft();
        void turnRight();
    

};