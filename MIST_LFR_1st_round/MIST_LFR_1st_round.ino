/*
  P controll using Weighted Average
*/

//#include <ResponsiveAnalogRead.h>
#define Kp 140
#define Kd 10
#define Ki 0

//PID error segment
float prevError = 0;
float error = 0;
float tError = 0;
float dError = 0;

//Sensor segment
int sensorData[7] = {0, 0, 0, 0, 0, 0, 0};
int midSensorValue[7] =  {0, 0, 0, 0, 0, 0, 0};

int countSensor = 0;
float sumSensor = 0;
float avgSensor = 0;

//Logic segment
bool leftTurn = 0;
bool rightTurn = 0;
bool pidControl = 0;
bool whiteLine = 0;
bool fullBlack = 0;
bool fullWhite = 0;
bool lineEnd = 0;

//Motor speed setup
float dSpeed = 0;
float leftSpeed = 0;
float rightSpeed = 0;
float baseSpeed = 100;
float maxSpeed = 200;//2X of baseSpeed

//Pin decleration
#define leftMotorUp 28
#define leftMotorDown 26
#define rightMotorUp 30
#define rightMotorDown 32

#define leftMotorPWM 5
#define rightMotorPWM 6

int fLineLED = 23;
int leftLED = 33;
int rightLED = 35;

const int sensors[7] = {A7, A6, A5, A4, A3, A2, A1};
#define irPin 34
#define rulePin 2
bool rule = 0; //0 for left, 1 for right

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 7; i++) pinMode(sensors[i], INPUT);

  pinMode(leftMotorUp, OUTPUT);
  pinMode(leftMotorDown, OUTPUT);
  pinMode(rightMotorUp, OUTPUT);
  pinMode(rightMotorDown, OUTPUT);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(rulePin, INPUT);
  pinMode(irPin, OUTPUT);

  pinMode(fLineLED, OUTPUT);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);

  digitalWrite(fLineLED, 1);
  calibrateIR();
  digitalWrite(fLineLED, 0);
}

//This function is for calibrating the IR sensors
//host: 'setup()'
void calibrateIR() {
  int white[7] = {0, 0, 0, 0, 0, 0, 0};
  int black[7] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
  leftSpeed = -75;
  rightSpeed = 75;

  for (int k = 0; k < 4; k++) {
    leftSpeed = -leftSpeed;
    rightSpeed = -rightSpeed;
    driveMotor();

    for (int j = 0; j < 32; j++) {
      if (k == 0 || k == 3) delay(12);
      else delay(25);

      for (int i = 0; i < 7; i++) {
        int a = 0, b = 0;

        digitalWrite(irPin, HIGH); //glow the emitter for 500 microseconds
        delayMicroseconds(500);
        a = analogRead(sensors[i]);

        digitalWrite(irPin, LOW); //turn off the emitter for 500 microseconds
        delayMicroseconds(500);
        b = analogRead(sensors[i]);

        sensorData[i] = a - b;

        white[i] = (sensorData[i] > white[i]) ? sensorData[i] : white[i];
        black[i] = (sensorData[i] < black[i]) ? sensorData[i] : black[i];
      }
    }
  }
  for (int i = 0; i < 7; i++) {
    midSensorValue[i] = (white[i] + black[i]) / 2;
  }
}
// Execution fuction
void loop() {

  readSensor();
  sensorAnalysis();

  if(lineEnd == 1) findWay();

  else if (pidControl == 1) setPID();

  else if (leftTurn == 1) setLeft();

  else if (rightTurn == 1) setRight();

  //----------------------------- partition

  if (pidControl == 0) resetPID();

  digitalWrite(fLineLED, whiteLine);
  digitalWrite(leftLED, leftTurn);
  digitalWrite(rightLED, rightTurn);
  driveMotor();
  printData();
}

//This function is for sensing the IR, SONAR & WHEEL ENCODER
//host: 'loop()'
void readSensor() {
  rule = digitalRead(rulePin);

  readIR();
  //readSonar();
}
//IR Sensing
//host: 'readSensor()'
void readIR() {
  sumSensor = 0;
  avgSensor = 0;

  countSensor = 0;
  for (int i = 0; i < 7; i++) {
    bool sensorActive = 0;

    digitalWrite(irPin, HIGH); //glow the emitter for 500 microseconds
    delayMicroseconds(500);
    int a = analogRead(sensors[i]);

    digitalWrite(irPin, LOW); //turn off the emitter for 500 microseconds
    delayMicroseconds(500);
    int b = analogRead(sensors[i]);
    int c =  a - b;

    //if ((whiteLine == 0 && c > midSensorValue[i]) || (whiteLine == 1 && c < midSensorValue[i])) { //if start at White Line
    if ((whiteLine == 0 && c < midSensorValue[i]) || (whiteLine == 1 && c > midSensorValue[i])) { //if Start at Black Line
      sensorData[i] = i - 3;
      sensorActive = 1;
    }
    else sensorData[i] = 0;

    sumSensor += sensorData[i];
    if (sensorActive == 1) countSensor++;
  }
  if (countSensor == 0) avgSensor = 0;
  else avgSensor = sumSensor / countSensor;
}

//This function analyzes sensor data
//host: 'loop()'
void sensorAnalysis() {
  if (countSensor == 0) {//White Surface
    fullWhite = 1;
    lineEnd = 0;  

    if(pidControl == 1){
      pidControl = 0;
      rightTurn = 0;
      leftTurn = 0;
      lineEnd = 1;      
    }
    else if(fullBlack == 1){
      fullBlack = 0;
      
      if(rightTurn == 1){
        pidControl = 0;
        leftTurn = 0;
      }
      else if(leftTurn == 1){
        pidControl = 0;
        rightTurn = 0;
      }
    }
    else{
      rightTurn = 0;
      leftTurn = 0;
      if (rule == 0) rightTurn = 1;
      else leftTurn = 1;
    }
  }
  else if (countSensor == 7) {// Black Surface -> Turning Left
    fullBlack = 1;
    pidControl = 0;
    rightTurn = 0;
    leftTurn = 0;

    if (rule == 0) leftTurn = 1;
    else rightTurn = 1;
  }
  else {// Line 
    if (rule == 0 && (sumSensor < 0 && (sensorData[0] == -3 || countSensor > 3))) { //Possible Left Way
      leftTurn = 1;
      pidControl = 0;
      rightTurn = 0;
    }
    else if (rule == 1 && (sumSensor > 0 && (sensorData[6] == 3 || countSensor > 3))) { //Possible Right Way
      rightTurn = 1;
      pidControl = 0;
      leftTurn = 0;
    }
    else if (countSensor == 0) ;

    else if(countSensor > 0 && (sensorData[0] == 0 || sensorData[6] == 0)){// forward line
      pidControl = 1;
      fullBlack = 0;
      leftTurn = 0;
      rightTurn = 0;
    }
  }
}

//This function resets the motor speeds using PID Algorithm
//host: 'loop()'
void resetPID() {
  error = 0;
  tError = 0;
}
//This function sets the motor speeds using PID Algorithm
//host: 'loop()'
void setPID() {
  prevError = error;
  error = avgSensor;
  dError = error - prevError;
  tError += error;

  dSpeed = Kp * error + Kd * dError; // + Ki * tError

  if (error > -1.5 && error < 1.5) baseSpeed = 150;
  else baseSpeed = 80;
  

  rightSpeed = baseSpeed - dSpeed;
  leftSpeed = baseSpeed + dSpeed;

  if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
  if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
  if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;
  if (leftSpeed < - maxSpeed) leftSpeed = -maxSpeed;
}

//This 2 functions set the motor speeds to turn
//host: 'loop()'
void setLeft() {
  //90deg turn at 0.448sec
  leftSpeed = -100;
  rightSpeed = 100;
}
void setRight() {
  leftSpeed = 100;
  rightSpeed = -100;
}
void findWay(){
  leftSpeed = 100;
  rightSpeed = 100;
  driveMotor();
  delay(100);
  leftSpeed = 0;
  rightSpeed = 0;
  driveMotor();

  if (countSensor <= 5 && sensorData[0] == -3 && sensorData[6] == 3){ // Line color Changed   
    //whiteLine = !whiteLine; 
    lineEnd = 0;
  }
  else if (countSensor == 0 || rule == 0) leftTurn = 1;
  else if (countSensor == 0 || rule == 1) rightTurn = 1;
  //lineEnd = 0;
}

//This function drives the motors
//host: 'loop()'
void driveMotor() {
  if (rightSpeed >= 0) {
    analogWrite(rightMotorPWM, rightSpeed);
    digitalWrite(rightMotorDown, LOW);
    digitalWrite(rightMotorUp, HIGH);
  }
  else {
    analogWrite(rightMotorPWM, -rightSpeed);
    digitalWrite(rightMotorUp, LOW);
    digitalWrite(rightMotorDown, HIGH);
  }
  if (leftSpeed >= 0) {
    analogWrite(leftMotorPWM, leftSpeed);
    digitalWrite(leftMotorDown, LOW);
    digitalWrite(leftMotorUp, HIGH);
  }
  else {
    analogWrite(leftMotorPWM, -leftSpeed);
    digitalWrite(leftMotorUp, LOW);
    digitalWrite(leftMotorDown, HIGH);
  }
}
//This function prints out all the data throughout the whole program
//host: 'loop()'
void printData() {
   //for(int i=0; i<7; i++){
  //Serial.print(sensorData[i]);
  //Serial.print("\t");
  //Serial.print("\t");
  //Serial.print(white[i]);
  //Serial.print("\t");
  /// Serial.print(black[i]);
  //}
  /*
    Serial.print(countSensor);
    Serial.print("\t");
    Serial.print(sumSensor);
    Serial.print("\t");
    Serial.print(avgSensor);
    Serial.print("\t");
    Serial.print(whiteLine);
    Serial.print("\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.print(leftSpeed);
    Serial.print("\t");
    Serial.print(rightSpeed);
    Serial.print("\t");
    Serial.print(fLine);
    Serial.print("\t");
    Serial.print(pidControl);
    Serial.print("\t");
  */

  //Serial.println("\t");
}
