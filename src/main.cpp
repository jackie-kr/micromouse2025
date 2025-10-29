#include <Arduino.h>
#include <pins.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include "LedDisplay.h"

#include "MPU9250.h"

#include <motor.h>
#include <sensor.h>

#include <string>

#include "chassis.h"
#include "maze.h"

Encoder frontRightEnc(ENCODER_FRONT_RIGHT_B, ENCODER_FRONT_RIGHT_A);
Encoder backRightEnc(ENCODER_BACK_RIGHT_A, ENCODER_BACK_RIGHT_B);
Encoder frontLeftEnc(ENCODER_FRONT_LEFT_B, ENCODER_FRONT_LEFT_A);
Encoder backLeftEnc(ENCODER_BACK_LEFT_B, ENCODER_BACK_LEFT_A);

MPU9250 mpu;

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

MiniPID anglePID(0, 000001, 0, 0);
MiniPID turnPID(0.01, 0.0000001, 0.15);
MiniPID distancePID(0.001, 0.00000001, 0);

LedDisplay display(DISPLAY_IN, DISPLAY_RS, DISPLAY_CLK, DISPLAY_CE, DISPLAY_RST, DISPLAY_LENGTH);

Location m_location;
Heading m_heading;

Maze maze;

String readline()
{
  String response = "";
  while (response == "")
  {
    response = Serial.readStringUntil('\n');
  }
  return response;
}

String communicate(String command)
{
  Serial.print(command + "\n");
  return readline();
}

bool getAck(String command)
{
  String response = communicate(command);
  return response == "ack";
}

bool getBoolean(String command)
{
  String response = communicate(command);
  return response == "true";
}

int getInteger(String command)
{
  String response = communicate(command);
  return response.toInt();
}

// Helper functions
void update_map()
{
  digitalWrite(EMITTER_LEFT_HALF, HIGH);
  digitalWrite(EMITTER_RIGHT_HALF, HIGH);
  bool leftWall = analogRead(IR_LEFT_ANGLE) >= 200;
  bool frontWall = analogRead(IR_LEFT_FRONT) >= 200;
  bool rightWall = analogRead(IR_RIGHT_ANGLE) >= 200;
  char w[] = "--- ";
  if (leftWall)
  {
    w[0] = 'L';
  };
  if (frontWall)
  {
    w[1] = 'F';
  };
  if (rightWall)
  {
    w[2] = 'R';
  };
  Serial.print(w);
  switch (m_heading)
  {
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
void log(String message)
{
  Serial.print("log " + message + "\n");
}

int mazeWidth()
{
  return getInteger("mazeWidth");
}

int mazeHeight()
{
  return getInteger("mazeHeight");
}

bool wallFront()
{
  return getBoolean("wallFront");
}

bool wallRight()
{
  return getBoolean("wallRight");
}

bool wallLeft()
{
  return getBoolean("wallLeft");
}

bool moveForward()
{
  return getAck("moveForward");
}

void turnRight()
{
  getAck("turnRight");
}

void turnLeft()
{
  getAck("turnLeft");
}

void setWall(int x, int y, char direction)
{
  Serial.print(
      "setWall " + String(x) + " " + String(y) + " " + String(direction) + "\n");
}

void clearWall(int x, int y, char direction)
{
  Serial.print(
      "clearWall " + String(x) + " " + String(y) + " " + String(direction) + "\n");
}

void setColor(int x, int y, char color)
{
  Serial.print(
      "setColor " + String(x) + " " + String(y) + " " + String(color) + "\n");
}

void clearColor(int x, int y)
{
  Serial.print(
      "clearColor " + String(x) + " " + String(y) + "\n");
}

void clearAllColor()
{
  Serial.print("clearAllColor\n");
}

void setText(int x, int y, String text)
{
  Serial.print(
      "setText " + String(x) + " " + String(y) + " " + text + "\n");
}

void clearText(int x, int y)
{
  Serial.print(
      "clearText " + String(x) + " " + String(y) + "\n");
}

void clearAllText()
{
  Serial.print("clearAllText\n");
}

bool wasReset()
{
  return getBoolean("wasReset");
}

void ackReset()
{
  getAck("ackReset");
}

// ----- Helpers -----

void move_ahead()
{
  chassis.moveForwardTile();
}

void turn_right()
{
  chassis.moveForwardTile();
  chassis.gyroTurnOrientation(90);
  m_heading = right_from(m_heading);
}

void turn_back()
{
  chassis.gyroTurnOrientation(180);
  chassis.moveForwardTile();
  m_heading = behind_from(m_heading);
}

void turn_left()
{
  // chassis.moveForwardTile();
  delay(200);
  for (int i = 0; i < 100; i++)
  {
    mpu.update();
  }
  chassis.gyroTurnOrientation(-90);
  m_heading = left_from(m_heading);
}

void search_maze()
{
  m_location = START;
  m_heading = NORTH;
  maze.initialise();
  maze.set_goal(Location(3, 3)); // TODO: This needs to be set based off of maze size

  maze.flood(maze.goal());
  while (m_location != maze.goal())
  {
    m_location = m_location.neighbour(m_heading);
    update_map();
    maze.flood(maze.goal());
    unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
    unsigned char hdgChange = (newHeading - m_heading) & 0x3;
    if (m_location != maze.goal())
    {
      switch (hdgChange)
      {
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
void setup()
{

  Wire1.setClock(400000);
  Serial.begin(115200);
  Wire1.begin();

  mpu.setup(0x68, MPU9250Setting(), Wire1);

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

  // pinMode(BUZZER, OUTPUT);

  // leftAngleIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_ANGLE);
  // leftAngleIR.initSensor();
  // leftFrontIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_FRONT);
  // leftFrontIR.initSensor();
  // rightAngleIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_ANGLE);
  // rightAngleIR.initSensor();
  rightFrontIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_FRONT);
  rightFrontIR.initSensor();

  chassis.setMotors(&backRightMotor, &backLeftMotor, &frontRightMotor, &frontLeftMotor);
  chassis.setChassisAttr(WHEEL_DIAMETER, ENCODER_TICKS_PER_WHEEL_ROTATION, WHEEL_TRACK);
  chassis.setPID(&distancePID, &anglePID, &turnPID);
  chassis.setMPU(&mpu);
  chassis.setError(7, 4);

  // Calibrate the MPU9250
  // delay(2000);
  // mpu.verbose(true); //Debugging purposes
  mpu.selectFilter(QuatFilterSel::MAHONY);
  mpu.setFilterIterations(20);
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
  mpu.setAccBias(-2.79, -27.39, -12.07);
  mpu.setGyroBias(3.41, -0.01, -0.35);
  mpu.setMagBias(-10.65, -3.55, -514.83);
  mpu.setMagScale(1.05, 0.99, 0.96);
  turnPID.setOutputLimits(0.4);
}

// Main loop
void loop()
{
  // Read button states (HIGH when pressed due to pull-down configuration)
  /*bool button1Pressed = digitalRead(BUTTON_1) == HIGH;
  bool button2Pressed = digitalRead(BUTTON_2) == HIGH;

  digitalWrite(EMITTER_RIGHT_HALF, HIGH);
  Serial.println(analogRead(IR_RIGHT_FRONT));
  digitalWrite(EMITTER_RIGHT_HALF, LOW);
  display.home();

  if (button1Pressed) {
    display.print("FWD ");
    frontRightMotor.setRawPWM(100, false);
    frontLeftMotor.setRawPWM(100, false);
    backRightMotor.setRawPWM(100, false);
    backLeftMotor.setRawPWM(100, false);
  } else if (button2Pressed) {
    display.print("REV ");
    frontRightMotor.setRawPWM(100, true);
    frontLeftMotor.setRawPWM(100, true);
    backRightMotor.setRawPWM(100, true);
    backLeftMotor.setRawPWM(100, true);
  } else {
    mpu.update();
    frontRightMotor.setRawPWM(0, false);
    frontLeftMotor.setRawPWM(0, false);
    backRightMotor.setRawPWM(0, false);
    backLeftMotor.setRawPWM(0, false);
    display.print(String(mpu.getYaw(), 2).substring(0, 4));
  }
*/
  log("test");
  delay(200); // Small delay to debounce and prevent flicker

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
