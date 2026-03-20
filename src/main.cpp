#include <Arduino.h>
#include <pins.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include "LedDisplay.h"

#include <Adafruit_BNO055.h>

#include <motor.h>
#include <sensor.h>

#include <string>

#include "chassis.h"
#include "maze.h"

#include <iostream>
#include <stack>
#include <cmath>



using namespace std;

// ─────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────
#define MAZE_SIZE 16

// Directions: 0=North, 1=East, 2=South, 3=West
int dr[] = { 1,  0, -1,  0};
int dc[] = { 0,  1,  0, -1};

// ─────────────────────────────────────────
//  Maze Grid & Mouse State
// ─────────────────────────────────────────
int maze[MAZE_SIZE][MAZE_SIZE];

int mouseR   = 0;
int mouseC   = 0;
int facing   = 0;   // 0=North, 1=East, 2=South, 3=West
int cameFrom = -1;

stack<int> pathStack;

// ─────────────────────────────────────────
//  Global Chassis Instance
//  Configure these with your actual hardware values
// ─────────────────────────────────────────
Chassis chassis;

// ─────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────
bool isGoal(int r, int c) {
    return (r == 7 || r == 8) && (c == 7 || c == 8);
}

bool inBounds(int r, int c) {
    return r >= 0 && r < MAZE_SIZE && c >= 0 && c < MAZE_SIZE;
}

int opposite(int d) { return (d + 2) % 4; }

// ─────────────────────────────────────────
//  Maze Initialization
// ─────────────────────────────────────────
void initMaze() {
    for (int i = 0; i < MAZE_SIZE; i++)
        for (int j = 0; j < MAZE_SIZE; j++)
            maze[i][j] = -1;
}

// ─────────────────────────────────────────
//  Physical Movement via Chassis
//  Replaces API::moveForward(), API::turnLeft(), API::turnRight()
// ─────────────────────────────────────────
void physicalMoveForward() {
    chassis.moveForwardTile();  // moves exactly one cell
}

void physicalTurnLeft() {
    chassis.turnLeft();         // 90° left turn
    facing = (facing + 3) % 4;
}

void physicalTurnRight() {
    chassis.turnRight();        // 90° right turn
    facing = (facing + 1) % 4;
}

// ─────────────────────────────────────────
//  Facing Utilities
// ─────────────────────────────────────────
void faceDirection(int target) {
    while (facing != target) {
        int diff = (target - facing + 4) % 4;
        if (diff == 1) { physicalTurnRight(); }
        else{
          physicalTurnLeft();  
        }
    }
}

// ─────────────────────────────────────────
//  Wall Sensing via Sensors
//  Replace hasWallFront/Left/Right with your actual sensor calls
//  e.g. sensor readings from sensor.cpp (IR or ToF distance sensors)
// ─────────────────────────────────────────
bool hasWallFront() {
    // TODO: Replace with actual sensor read from sensor.cpp
    // Example: return sensor.getDistanceFront() < WALL_THRESHOLD;
    return analogRead(IR_LEFT_FRONT) >= 200;
    
    
}

bool hasWallLeft() {
    // TODO: Replace with actual sensor read from sensor.cpp
    // Example: return sensor.getDistanceLeft() < WALL_THRESHOLD;
    return analogRead(IR_LEFT_FRONT) >= 200;
}

bool hasWallRight() {
    // TODO: Replace with actual sensor read from sensor.cpp
    // Example: return sensor.getDistanceRight() < WALL_THRESHOLD;
    return analogRead(IR_RIGHT_ANGLE) >= 200
}

bool wallInDirection(int d) {
    int diff = (d - facing + 4) % 4;
    switch (diff) {
        case 0: return hasWallFront();
        case 1: return hasWallRight();
        case 3: return hasWallLeft();
        case 2: {
            // Check behind: turn around, sense, turn back
            physicalTurnRight(); physicalTurnRight();
            bool w = hasWallFront();
            physicalTurnRight(); physicalTurnRight();
            return w;
        }
    }
    return true;
}

// ─────────────────────────────────────────
//  Path Scanning
// ─────────────────────────────────────────
int scanPaths(int r, int c, int from) {
    int count = 0;
    for (int d = 0; d < 4; d++) {
        if (d == from) continue;
        if (!wallInDirection(d)) count++;
    }
    return count;
}

// ─────────────────────────────────────────
//  Distance to goal center (7.5, 7.5)
// ─────────────────────────────────────────
float distToGoal(int r, int c) {
    float ddr = r - 7.5f;
    float ddc = c - 7.5f;
    return ddr * ddr + ddc * ddc;
}

// ─────────────────────────────────────────
//  Exploration Algorithm
// ─────────────────────────────────────────
void explore() {
    int r = mouseR, c = mouseC;
    int from = cameFrom;

    while (!isGoal(r, c)) {

        // 1. Scan if unvisited
        if (maze[r][c] == -1) {
            int paths = scanPaths(r, c, from);
            maze[r][c] = paths;
        }

        // 2. Collect unvisited neighbors, pick closest to goal
        int chosen = -1;
        float bestDist = 1e9;
        int checkOrder[] = {
            (facing + 3) % 4,  // Left
            facing,            // Forward
            (facing + 1) % 4   // Right
        };

        for (int i = 0; i < 3; i++) {
            int d = checkOrder[i];
            if (d == from) continue;
            int nr = r + dr[d];
            int nc = c + dc[d];
            if (inBounds(nr, nc) && !wallInDirection(d) && maze[nr][nc] == -1) {
                float dist = distToGoal(nr, nc);
                if (dist < bestDist) {
                    bestDist = dist;
                    chosen = d;
                }
            }
        }

        // 3. Found unvisited neighbor — move there
        if (chosen != -1) {
            pathStack.push(opposite(chosen));
            faceDirection(chosen);
            physicalMoveForward();      // <-- Physical movement
            r     += dr[chosen];
            c     += dc[chosen];
            from   = opposite(chosen);
            facing = chosen;
            continue;
        }

        // 4. Dead end — backtrack using stack
        if (pathStack.empty()) {
            cerr << "No path to goal found." << endl;
            return;
        }

        int backDir = pathStack.top();
        pathStack.pop();

        faceDirection(backDir);
        physicalMoveForward();          // <-- Physical backtrack movement

        facing = backDir;
        from   = opposite(backDir);
        r += dr[backDir];
        c += dc[backDir];

        if (maze[r][c] > 0) {
            maze[r][c]--;
        }
    }

    cerr << "Goal reached at (" << r << "," << c << ")!" << endl;
}

// ─────────────────────────────────────────
//  Entry Point
// ─────────────────────────────────────────
int main() {
    // ── Configure chassis with your hardware values ──
    // chassis.setChassisAttr(wheelDiameter, encRatio, wheelTrack);
    // chassis.setMotors(&backRightMotor, &backLeftMotor, &frontRightMotor, &frontLeftMotor);
    // chassis.setPID(&distPID, &anglePID, &turnPID);
    // chassis.setError(distanceError, angleError);

    cerr << "Micromouse starting..." << endl;
    initMaze();
    explore();
    return 0;
}

Encoder frontRightEnc(ENCODER_FRONT_RIGHT_B, ENCODER_FRONT_RIGHT_A);
Encoder backRightEnc(ENCODER_BACK_RIGHT_A, ENCODER_BACK_RIGHT_B);
Encoder frontLeftEnc(ENCODER_FRONT_LEFT_B, ENCODER_FRONT_LEFT_A);
Encoder backLeftEnc(ENCODER_BACK_LEFT_B , ENCODER_BACK_LEFT_A);

Adafruit_BNO055 myIMU;
//asdfasdf
Sensor leftAngleIR;
Sensor rightAngleIR;

Sensor leftFrontIR;
Sensor rightFrontIR;

Motor frontLeftMotor;
Motor frontRightMotor;
Motor backLeftMotor;
Motor backRightMotor;

MiniPID frontLeftPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);
MiniPID frontRightPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);
MiniPID backLeftPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);
MiniPID backRightPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);

Chassis chassis;

MiniPID anglePID(0,000001,0,0);
MiniPID turnPID(0.01,0.0000001,0.15);
MiniPID distancePID(0.001,0.00000001,0);

LedDisplay display(DISPLAY_IN, DISPLAY_RS, DISPLAY_CLK, DISPLAY_CE, DISPLAY_RST, DISPLAY_LENGTH);

Location m_location;
Heading m_heading;

Maze maze; 

// Helper functions
void update_map() {
  digitalWrite(EMITTER_LEFT_HALF, HIGH);
  digitalWrite(EMITTER_RIGHT_HALF, HIGH);
  bool leftWall = analogRead(IR_LEFT_ANGLE) >= 200;
  bool frontWall = analogRead(IR_LEFT_FRONT) >= 200;
  bool rightWall = analogRead(IR_RIGHT_ANGLE) >= 200;
  char w[] = "--- ";
  if (leftWall) {
    w[0] = 'L';
  };
  if (frontWall) {
    w[1] = 'F';
  };
  if (rightWall) {
    w[2] = 'R';
  };
  Serial.print(w);
  switch (m_heading) {
    case NORTH:
      maze.update_wall_state(m_location, NORTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, leftWall ? WALL : EXIT);
      break;
    case EAST:
      maze.update_wall_state(m_location, EAST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, leftWall ? WALL : EXIT);
      break;
    case SOUTH:
      maze.update_wall_state(m_location, SOUTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, leftWall ? WALL : EXIT);
      break;
    case WEST:
      maze.update_wall_state(m_location, WEST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, leftWall ? WALL : EXIT);
      break;
    default:
      // This is an error. We should handle it.
      break;
  }
}

void move_ahead(){
  chassis.moveForwardTile();
}

void turn_right(){
  chassis.moveForwardTile();
  chassis.gyroTurnOrientation(90);
  m_heading = right_from(m_heading);
}

void turn_back(){
  chassis.gyroTurnOrientation(180);
  chassis.moveForwardTile();
  m_heading = behind_from(m_heading);
}

void turn_left(){
  // chassis.moveForwardTile();
  delay(200);
  for(int i = 0; i < 100; i++){
     //mpu.update();
  }
  chassis.gyroTurnOrientation(-90);
  m_heading = left_from(m_heading);
}

void search_maze(){
  m_location = START;
  m_heading = NORTH;
  maze.initialise();
  maze.set_goal(Location(3,3)); // TODO: This needs to be set based off of maze size

  maze.flood(maze.goal());
  while (m_location != maze.goal()){
    m_location = m_location.neighbour(m_heading);
    update_map();
    maze.flood(maze.goal());
    unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
    unsigned char hdgChange = (newHeading - m_heading) & 0x3;
    if (m_location != maze.goal()) {
      switch (hdgChange) {
        // each of the following actions will finish with the
        // robot moving and at the sensing point ready for the
        // next loop iteration
        case AHEAD:
          move_ahead();
          break;
        case RIGHT:
          turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          turn_left();
          break;
      }
    }
  }
}

// Setup the mouse
void setup() {
  
  Wire1.setClock(400000);
  Serial.begin(115200);
  Wire1.begin();

  //mpu.setup(0x68, MPU9250Setting(), Wire1);
  myIMU = Adafruit_BNO055(55, 0x28, &Wire1);
  myIMU.begin();
  myIMU.setExtCrystalUse(true);

  display.begin();
  display.home();
  display.print("INIT");
  delay(1000);

  distancePID.setOutputRampRate(0.01);

    pinMode(BUTTON_1, INPUT);
    pinMode(BUTTON_2, INPUT);

    pinMode(MOTOR_LEFT_IN_1, OUTPUT);
    pinMode(MOTOR_LEFT_IN_2, OUTPUT);
    pinMode(MOTOR_BACK_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_FRONT_LEFT_PWM, OUTPUT);

    pinMode(MOTOR_RIGHT_IN_1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN_2, OUTPUT);
    pinMode(MOTOR_BACK_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_FRONT_RIGHT_PWM, OUTPUT);

    frontLeftMotor.setMotorPins(MOTOR_LEFT_IN_1, MOTOR_LEFT_IN_2, MOTOR_FRONT_LEFT_PWM);
    frontLeftMotor.setMotorAttr(MAX_VELOCITY);
    frontLeftMotor.setEncoder(&frontLeftEnc);
    frontLeftMotor.setVelPID(&frontLeftPID);
    frontLeftMotor.initMotor();

    backLeftMotor.setMotorPins(MOTOR_LEFT_IN_1, MOTOR_LEFT_IN_2, MOTOR_BACK_LEFT_PWM);
    backLeftMotor.setMotorAttr(MAX_VELOCITY);
    backLeftMotor.setEncoder(&backLeftEnc);
    backLeftMotor.setVelPID(&backLeftPID);
    backLeftMotor.initMotor();

    frontRightMotor.setMotorPins(MOTOR_RIGHT_IN_1, MOTOR_RIGHT_IN_2, MOTOR_FRONT_RIGHT_PWM);
    frontRightMotor.setMotorAttr(MAX_VELOCITY);
    frontRightMotor.setEncoder(&frontRightEnc);
    frontRightMotor.setVelPID(&frontRightPID);
    frontRightMotor.initMotor();

    backRightMotor.setMotorPins(MOTOR_RIGHT_IN_1, MOTOR_RIGHT_IN_2, MOTOR_BACK_RIGHT_PWM);
    backRightMotor.setMotorAttr(MAX_VELOCITY);
    backRightMotor.setEncoder(&backRightEnc);
    backRightMotor.setVelPID(&backRightPID);
    backRightMotor.initMotor();

    //pinMode(BUZZER, OUTPUT);

    leftAngleIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_ANGLE);
    leftAngleIR.initSensor();
    leftFrontIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_FRONT);
    leftFrontIR.initSensor();
    rightAngleIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_ANGLE);
    rightAngleIR.initSensor();
    rightFrontIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_FRONT);
    rightFrontIR.initSensor();

    chassis.setMotors(&backRightMotor, &backLeftMotor, &frontRightMotor, &frontLeftMotor);
    chassis.setChassisAttr(WHEEL_DIAMETER, ENCODER_TICKS_PER_WHEEL_ROTATION, WHEEL_TRACK);
    chassis.setPID(&distancePID, &anglePID, &turnPID);
    chassis.setIMU(&myIMU);
    chassis.setError(7, 4);

    // Calibrate the MPU9250
    // delay(2000);
    // mpu.verbose(true); //Debugging purposes
    //mpu.selectFilter(QuatFilterSel::MAHONY);
    // mpu.setFilterIterations(20);
    // Serial.print("accel setup\n");
    // mpu.calibrateAccelGyro();
    // frontRightMotor.setRawPWM(100, false);
    // delay(500);
    // frontRightMotor.stop();
    // Serial.print("mag setup\n");
    // delay(1000);
    // mpu.calibrateMag();
    // Serial.print("setup done\n");
    // frontLeftMotor.setRawPWM(100, false);
    // delay(500);
    // frontLeftMotor.stop();
  
    // Set biases
    //mpu.setAccBias(-2.79, -27.39, -12.07);
    //mpu.setGyroBias(3.41, -0.01, -0.35);
    //mpu.setMagBias(-10.65, -3.55, -514.83);
    //mpu.setMagScale(1.05, 0.99, 0.96);
    turnPID.setOutputLimits(0.4);
    //chassis.moveForwardTile();
   //pinMode(2, OUTPUT);
}

// Main loop
void loop() {
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroVal = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  uint8_t system, gyro, accel, mag = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mag);
  // ---- LEFT SIDE ----
  digitalWrite(EMITTER_LEFT_HALF, LOW);
  delayMicroseconds(200);
  int leftAngle_ambient = analogRead(IR_LEFT_ANGLE);
  int leftFront_ambient = analogRead(IR_LEFT_FRONT);

  digitalWrite(EMITTER_LEFT_HALF, HIGH);
  delayMicroseconds(200);
  int leftAngle_active = analogRead(IR_LEFT_ANGLE);
  int leftFront_active = analogRead(IR_LEFT_FRONT);

  int leftAngle = leftAngle_active - leftAngle_ambient;
  int leftFront = leftFront_active - leftFront_ambient;

  digitalWrite(EMITTER_LEFT_HALF, LOW);

  // ---- RIGHT SIDE ----
  digitalWrite(EMITTER_RIGHT_HALF, LOW);
  delayMicroseconds(200);
  int rightAngle_ambient = analogRead(IR_RIGHT_ANGLE);
  int rightFront_ambient = analogRead(IR_RIGHT_FRONT);

  digitalWrite(EMITTER_RIGHT_HALF, HIGH);
  delayMicroseconds(200);
  int rightAngle_active = analogRead(IR_RIGHT_ANGLE);
  int rightFront_active = analogRead(IR_RIGHT_FRONT);

  int rightAngle = rightAngle_active - rightAngle_ambient;
  int rightFront = rightFront_active - rightFront_ambient;

  digitalWrite(EMITTER_RIGHT_HALF, LOW);

  // ---- PRINT ----
  Serial.print("LA: "); Serial.print(leftAngle);
  Serial.print("  LF: "); Serial.print(leftFront);
  Serial.print("  RA: "); Serial.print(rightAngle);
  Serial.print("  RF: "); Serial.println(rightFront);

  delay(200);
  /*Serial.print("Calibration: Sys="); Serial.print(system, DEC);
  Serial.print(" Gyro="); Serial.print(gyro, DEC);
  Serial.print(" Accel="); Serial.print(accel, DEC);  Serial.print(" Mag="); Serial.print(mag, DEC);
  Serial.print("Accel X: "); Serial.print(acc.x());
  Serial.print(" Accel Y: "); Serial.print(acc.y());
  Serial.print(" Accel Z: "); Serial.print(acc.z());
  Serial.print("Gyro X: "); Serial.print(gyroVal.x());
  Serial.print(" Gyro Y: "); Serial.print(gyroVal.y());
  Serial.print(" Gyro Z: "); Serial.println(gyroVal.z());*/

  /*chassis.turnRight();
  delay(1000);
  chassis.turnLeft();
  delay(1000);
  chassis.moveForwardTile();
  delay(1000);*/
  //digitalWrite(2, HIGH);
  /*
  // Read button states (HIGH when pressed due to pull-down configuration)
  bool button1Pressed = digitalRead(BUTTON_1) == HIGH;
  bool button2Pressed = digitalRead(BUTTON_2) == HIGH;

  display.print("Runs");
  
  digitalWrite(EMITTER_RIGHT_HALF, HIGH);
  Serial.println(analogRead(IR_RIGHT_FRONT));
  digitalWrite(EMITTER_RIGHT_HALF, LOW);
  display.home();

  Serial.println(leftFrontIR.getDistance());

  if (button1Pressed) {
    //display.print("FWD ");
    frontRightMotor.setRawPWM(100, false);
    frontLeftMotor.setRawPWM(100, false);
    backRightMotor.setRawPWM(100, false); 
    backLeftMotor.setRawPWM(100, false);
    display.print(backRightMotor.getEncoder());
  } else if (button2Pressed) {
    display.print("REV ");
    frontRightMotor.setRawPWM(100, true);
    frontLeftMotor.setRawPWM(100, true);
    backRightMotor.setRawPWM(100, true);
    backLeftMotor.setRawPWM(100, true);
  } else {
    mpu.update();
    frontRightMotor.setRawPWM(100, false);
    frontLeftMotor.setRawPWM(0, false);
    backRightMotor.setRawPWM(100, false); 
    backLeftMotor.setRawPWM(0, false);
    display.print(String(mpu.getYaw(), 2).substring(0, 4));
    
  }*/

  //Serial.println("encoder: " + String(backLeftMotor.getEncoder()));
  //search_maze();
  /*delay(200); // Small delay to debounce and prevent flicker

  //Serial.println("Left Front IR: " + String(leftFrontIR.getDistance()));
  //Serial.println("Right Front IR: " + String(rightFrontIR.getDistance()));
  delay(200);
  frontLeftMotor.setRawPWM(100, true);
  //backLeftMotor.setRawPWM(100, true);
  backRightMotor.setRawPWM(100, true);
  frontRightMotor.setRawPWM(100, true);
  delay(2000);

  

  frontLeftMotor.stop();
  //backLeftMotor.stop();
  backRightMotor.stop();
  frontRightMotor.stop();
  delay(2000);*/

  // // Use IR to start the mouse
  // digitalWrite(EMITTER_LEFT_HALF, HIGH);
  // while(900>analogRead(IR_LEFT_FRONT)){
  //   delay(10);
  //   mpu.update();
  // }
  // Serial.printf("begin\n");
  // delay(1000);

  // mpu.update();
  // Serial.print(mpu.getYaw()); Serial.print(", ");
  // Serial.print(mpu.getPitch()); Serial.print(", ");
  // Serial.print(mpu.getRoll()); Serial.print("\n");

  // display.home();
  // display.print(String(mpu.getYaw(), 2).substring(0, 4));

    /*Serial.printf("IMU test\n");
    while(true){
        // IMU.getGyro(&gyroData);
        // auto result = mySensor.magUpdate();
        // if (result != 0) {
        //   mySensor.beginMag();
        // result = mySensor.magUpdate();
        // }
        // mySensor.magUpdate();
        // mySensor.gyroUpdate();
        // mySensor.accelUpdate();
        
        // Serial.printf("Mag X:%f Y:%f Z:%f\n", mySensor.magX(), mySensor.magY(), mySensor.magZ());
        // Serial.printf("Gyro X:%f Y:%f Z:%f\n", mySensor.gyroX(), mySensor.gyroY(), mySensor.gyroZ());
        // Serial.printf("Accel X:%f Y:%f Z:%f\n", mySensor.accelX(), mySensor.accelY(), mySensor.accelZ());
        if (mpu.update()) {
          Serial.print(mpu.getYaw()); Serial.print(", ");
          Serial.print(mpu.getPitch()); Serial.print(", ");
          Serial.println(mpu.getRoll()); Serial.print("\n");
      }*/

      /*digitalWrite(EMITTER_LEFT_HALF, HIGH);
      while(1000>analogRead(IR_LEFT_FRONT)){
        delay(10);
      }

      delay(500);
      chassis.moveForwardTile();
     
      chassis.turnRight();

      chassis.moveForwardTile();
      chassis.moveForwardTile();

      chassis.turnRight();

      chassis.moveForwardTile();

      chassis.turnLeft();

      chassis.moveForwardTile();

      Serial.printf("help\n");*/
      
        

}