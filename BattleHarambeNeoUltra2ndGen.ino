/*
  Battle Harambe Neo Ultra-2nd Gen
  Rev.0

 created 1 Dec. 2016
 by Phoenix

 */
 #include <NewPing.h>
 /*
  * ======================
  * Fields Declarations 
  * ======================
 */

// MOTARS PINS
const int MOTOR_A1 = 10;
const int MOTOR_A2 = 9;
const int MOTOR_B1 = 6;
const int MOTOR_B2 = 5;

// LIGHT SENSOR
const int LIGHT_SENSOR_PIN = A4;  // Analog input pin that the LIGHT_SENSOR is attached to
const int LIGHT_SENSOR_NUM_READ = 3; // Samplesize for smoothing the read
const int LIGHT_SENSOR_PING_INTERVAL = 2;
int lightSensorReadIndex = 0;
int lightSensorValueSum = 0;
int lightSensorValue = 1; // The sensor value meant to be used.
int lightSensorPingTimer = 0;

// TOUCH SENSOR
const int TOUCH_SENSOR_PIN = A0;  // Analog input pin that the TOUCH_SENSOR is attached to
const int TOUCH_SENSOR_NUM_READ = 3; // Samplesize for smoothing the read
const int TOUCH_SENSOR_PING_INTERVAL = 2;
int touchSensorReadIndex = 0;
int touchSensorValueSum = 0;
int touchSensorValue = 100; // The sensor value meant to be used.
int touchSensorPingTimer = 0;

// DISTANCE SONAR SENSOR
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;
const int MAX_DISTANCE = 200;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sonar object

const int SONAR_PING_INTERVAL = 3;
const int SONAR_NUM_READ = 3; // Samplesize for smoothing the read
int sonarReadIndex = 0;
int distanceSum = 0;
int distance = 1; // The sensor value meant to be used.
int sonarPingTimer = 0;

// PRINT TIMER
const int PRINT_INTERVAL = 50;
int printTimer = 0;

// FLAGS
int adrenalineFlag = 0; // Flag to trigger adrenalineRush attack [6 for initialize an attack]
int enermyOnSightFlag = 0; // Flag if enermy detected in sight
int scanFlag = 0; // Flag to perform a scan
int OUT_OF_RING_FLAG = 0;// Critical Flag to show out of RING [6 for initialize an emegency move]

// *** Movement Control ***
int speed = 0;
unsigned char prevMovingState = 0;
unsigned char movingState = 0; // bytewise Moving State: [n/a][breakingBit][movingBit][turingBit]
                                        //                   0b0010 1001 - forward
                                        //                   0b0010 0110 - backward    
                                        //                   0b0100 1010 - left turn
                                        //                   0b0100 0101 - right turn
                                        //                   0b0001 0000 - breaking 
                                        //                   0b0000 0000 - stoped
const unsigned char FORWARD = 0x9;
const unsigned char BACKWARD = 0x6;
const unsigned char LEFTTURN= 0xa;
const unsigned char RIGHTTURN = 0x5;
const unsigned char STOP = 0x0;

const unsigned char MOVING_MASK = 0x20;
const unsigned char TURNING_MASK = 0x40;
const unsigned char BREAKING_MASK = 0x10;

// Count for cruising performed;
int cruisingCount = 0;
const int MAX_CRUISING_COUNTS = 3;


const int DECISION_INTERVAL = 2;
int decisionTimer = 0;

// Emergency
const int EMERGENCY_REVERSE_DURATION = 400;
const int EMERGENCY_TURN_DURATION = 200;
const int EMERGENCY_INTERVAL = 5;
unsigned char emergencyDirection = BACKWARD; 
int emegencyTimer = 0;

// BREAKER
const int BREAKER_INTERVAL = 3;
unsigned char breakerDirection = BACKWARD; 
int breakerFlag = 0;
int breakerTimer = 0;


// DASH
const int DASH_DURATION = 500;
const int DASH_INTERVAL = 2;

// ADRENALINE
const int ADRENALINE_TURN_DURATION = 400;
const int ADRENALINE_INTERVAL = 5;


// SCAN
const int SCAN_TURN_DURATION = 400;
const int SCAN_INTERVAL = 5;

// GENERAL MOVE VARS
int generalMoveState = FORWARD;
int interval = 2;
int duration = 0;
int moveTimer = 0;

// NORMAL CYCLE
const int NORMAL_CYCLE_INTERVAL = 3;
int nomalCycleTimer = 0;

 /*
  * ======================
  * Basic Sensor Reading Functions 
  * ======================
  */
// int getLightSensorReading(){
//   return analogRead(LIGHT_SENSOR_PIN);
// }

// int getTouchSensorReading(){
//   return analogRead(TOUCH_SENSOR_PIN);
// }

// int getSonarReading(){
//   return sonar.ping_cm();
// }

// // read sensors and make an average value output
// int readSensor(int (*getReading)(), int* output, int* readIndex, int* sum, int* pingTimer, int INTERVAL, int NUMREAD){
//   if (*pingTimer < millis()){
//     *sum += getReading();
//     *readIndex++;

//     *pingTimer = millis() + INTERVAL;

//     if (*readIndex == NUMREAD){
//       *readIndex = 0;
//       *output = *sum / NUMREAD;
//       *sum = 0;
//       return 1; // return 1 if output refreshed.
//     }
//   }
//   return 0;
// }

int readLightSensor(){
  if (lightSensorPingTimer < millis()){
    lightSensorPingTimer = millis() + LIGHT_SENSOR_PING_INTERVAL;
    lightSensorValueSum += analogRead(LIGHT_SENSOR_PIN);
    if (++lightSensorReadIndex == LIGHT_SENSOR_NUM_READ){
      lightSensorReadIndex = 0;
      lightSensorValue = lightSensorValueSum/LIGHT_SENSOR_NUM_READ;
      lightSensorValueSum = 0;
      return 1;
    }
  }
  return 0;
}

int readTouchSensor(){
  if (touchSensorPingTimer < millis()){
    touchSensorPingTimer = millis() + TOUCH_SENSOR_PING_INTERVAL;
    touchSensorValueSum += analogRead(TOUCH_SENSOR_PIN);
    if (++touchSensorReadIndex == TOUCH_SENSOR_NUM_READ){
      touchSensorReadIndex = 0;
      touchSensorValue = touchSensorValueSum/TOUCH_SENSOR_NUM_READ;
      touchSensorValueSum = 0;
      return 1;
    }
  }
  return 0;
}

int readSonar(){
  if (sonarPingTimer < millis()){
    sonarPingTimer = millis() + SONAR_PING_INTERVAL;
    distanceSum += sonar.ping_cm();
    if (++sonarReadIndex == SONAR_NUM_READ){
      sonarReadIndex = 0;
      distance = distanceSum/SONAR_NUM_READ;
      distanceSum = 0;
      return 1;
    }
  }
  return 0;
}

 /*
  * ======================
  * Advanced Movement Functions 
  * ======================
 */
int refreshLightSensor(){
  if(readLightSensor()){
    if (lightSensorValue > 1){
      if (OUT_OF_RING_FLAG == 0){
        OUT_OF_RING_FLAG = 6;
        if (++cruisingCount > MAX_CRUISING_COUNTS){
          scanFlag = 6;
        }
      }
      return 3;
    }
  }
  return 0;
}

int refreshTouchSensor(){
  if(readTouchSensor()){
    if (touchSensorValue == 0){
      if (adrenalineFlag == 0){
        adrenalineFlag = 6;  
      }
      return 5;
    }  
  }
  return 0;
}

int refreshSonar(){
  if(readSonar()){
    if (distance < 35 && enermyOnSightFlag == 0){
      adrenalineFlag = 0;
      enermyOnSightFlag = 1;
      return 1;
    } else {
      enermyOnSightFlag = 0;
    }
  }
  return 0;
}

// Refresh all sensor stats and set flags if needed.
// return states: retVal & 0xFF       : return from LightSensor
//                retVal>>8 & 0xFF    : return from Sonar
//                retVal>>16 & 0xFF   : return from TouchSensor
int refreshAllSensors(){
  int retVal = 0;
  retVal |= refreshLightSensor() & 0xFF;
  retVal |= (refreshSonar()<<8) & 0xFF;
  retVal |= (refreshTouchSensor()<<16) & 0xFF;

  printSensorStats(retVal); // May increase performance if commented.
  return retVal;
}

// print the results to the serial monitor:
void printSensorStats(int sensorReturnStats){
  if (printTimer < millis()){
    Serial.print("Millis() = ");
    Serial.print(millis());
    Serial.print(",\t Sensor Return Stats = 0x");
    Serial.println(sensorReturnStats, BIN);
    Serial.print("Light sensor = ");
    Serial.print(lightSensorValue);
    Serial.print("\t Touch sensor = ");
    Serial.print(touchSensorValue);
    Serial.print("\t Sonar sensor = ");
    Serial.println(distance);
  }
}

 /*
  * ======================
  * Basic Movement Control 
  * ======================
 */
// mask used with FORWARD, BACKWARD, LEFTTURN, RIGHTTURN, STOP
void basicMove(unsigned char mask, int spd){
  analogWrite(MOTOR_A1, spd*((mask>>3)&0x1));
  analogWrite(MOTOR_A2, spd*((mask>>2)&0x1));
  analogWrite(MOTOR_B1, spd*((mask>>1)&0x1));
  analogWrite(MOTOR_B2, spd*(mask&0x1));
}


 /*
  * ======================
  * Advanced Movement Functions
  * ======================
 */

// set the movingState, return 1 if newState changed from prevMovingState 
int setMovingState(unsigned char newState, int spd){
  if (newState != prevMovingState || spd != speed){
    movingState = newState;
    speed = spd;
    return 1;
  }
  return 0;
}

// parameter initializer for generalMove
void initGeneralMove(int dur, int itv, unsigned char mS, int spd){
  duration = dur;
  interval = itv;
  // Setting up proper moveState
  mS = mS & 0xF;
  switch(mS){
    case(FORWARD):
    case(BACKWARD):
      mS = (mS | MOVING_MASK);
      break;
    case(LEFTTURN):
    case(RIGHTTURN):
      mS = (mS | TURNING_MASK);
      break;
    default:
      mS = STOP;
  }
  generalMoveState = mS;
  speed = spd;
}

// General move for duration, must set duration and interval with initGeneralMove first
// Return if moving state changed 
int generalMove(){
  int retVal = 0;
  if (moveTimer < millis()){
    if (duration > 0)
      retVal = setMovingState(generalMoveState, speed);
    duration--;
    moveTimer = millis() + interval;  
  }
  return retVal;
}

// Breaker iniliazer
void initBreaker(){
  breakerFlag = 3;
  if(prevMovingState & MOVING_MASK)
    breakerDirection = (~prevMovingState) & 0xF;
  else
    breakerDirection = BACKWARD;
}

// 3 step breaker
// Return if moving state changed 
int breaker(){
  int retVal = 0;
  if (breakerFlag != 0 && breakerTimer < millis()){
    switch (breakerFlag--){
      case 3: // Stop
        retVal = setMovingState((STOP|BREAKING_MASK), 0);
        break;
      case 2: // Reverse 
        retVal = setMovingState((emergencyDirection|BREAKING_MASK), 255);
        break;
      case 1: // Stop
        retVal = setMovingState((STOP|BREAKING_MASK), 0);
        break;
      default:
        breakerFlag = 0;
        retVal = setMovingState(STOP, 0);
    }
    breakerTimer = millis() + BREAKER_INTERVAL;
  }
  return retVal;
}


 /*
  * ======================
  * Strategic Handling Functions
  * ======================
 */

// Desision control for emgency
int emegencyHandler(){
  int retVal = 0;
  switch(OUT_OF_RING_FLAG){
    case 6: // set the reversed direction and breakerFlag
      initBreaker();
      emergencyDirection = breakerDirection;
      OUT_OF_RING_FLAG--;
    case 5: // Breaker
      retVal = breaker();
      if (breakerFlag == 0)
        OUT_OF_RING_FLAG--;
      break;
    case 4: // Reverse
      initGeneralMove(EMERGENCY_REVERSE_DURATION, EMERGENCY_INTERVAL, emergencyDirection, 255);
      OUT_OF_RING_FLAG--;
    case 3:
      retVal = generalMove();
      if (duration == 0)
        OUT_OF_RING_FLAG--;
      break;
    case 2: // Turn Left
      initGeneralMove(EMERGENCY_TURN_DURATION, EMERGENCY_INTERVAL, LEFTTURN, 255);
      OUT_OF_RING_FLAG--;
    case 1:
      retVal = generalMove();
      if (duration == 0){
        OUT_OF_RING_FLAG--;
        retVal = setMovingState(STOP, 0);
      }
      break;
    default:
      OUT_OF_RING_FLAG = 0;
  }
  return retVal;
}

// Dash control
int dash(){
  int retVal = 0;
  switch (enermyOnSightFlag){
    case 2:
      initGeneralMove(DASH_DURATION, DASH_INTERVAL, FORWARD, 255);
      enermyOnSightFlag--;
    case 1:
      retVal = generalMove();
      if (duration == 0){
        enermyOnSightFlag--;
        retVal = setMovingState(STOP, 0);
      }
      break;
    default:
      enermyOnSightFlag = 0;
  }
  return retVal;
}

// adrenalineRush control
int adrenalineRush(){
  int retVal = 0;
  switch (adrenalineFlag){
    case 6:
      initGeneralMove(DASH_DURATION, DASH_INTERVAL, FORWARD, 255);
      adrenalineFlag--;
    case 5:
      retVal = generalMove();
      if (duration == 0)
        adrenalineFlag--;
      break;
    case 4:
      initBreaker();
      adrenalineFlag--;
    case 3:
      retVal = breaker();
      if (breakerFlag == 0)
        adrenalineFlag--;
      break;
    case 2:
      initGeneralMove(ADRENALINE_TURN_DURATION, ADRENALINE_INTERVAL, RIGHTTURN, 255);
      adrenalineFlag--;
    case 1:
      retVal = generalMove();
      if (duration == 0){
        adrenalineFlag--;
        retVal = setMovingState(STOP, 0);
      }
      break;
    default:
      adrenalineFlag = 0;
  }
  return retVal;
}


int scan(){
  int retVal = 0;
  switch (scanFlag){
    case 6:
      initGeneralMove(SCAN_TURN_DURATION, SCAN_INTERVAL, LEFTTURN, 255);
      scanFlag--;
    case 5:
      retVal = generalMove();
      if (duration == 0)
        scanFlag--;
      break;
    case 4:
      initBreaker();
      scanFlag--;
    case 3:
      retVal = breaker();
      if (breakerFlag == 0)
        scanFlag--;
      break;
    case 2:
      initGeneralMove(SCAN_TURN_DURATION/2, SCAN_INTERVAL, RIGHTTURN, 255);
      scanFlag--;
    case 1:
      retVal = generalMove();
      if (duration == 0){
        scanFlag--;
        retVal = setMovingState(STOP, 0);
      }
      break;
    default:
      scanFlag = 0;
  }
  return retVal;
}

int normalCycle(){
  resetFlags();
  int retVal = setMovingState(FORWARD, 200);
  return retVal;
}

int decisionMaking(){
  if (decisionTimer < millis()){
    if (OUT_OF_RING_FLAG != 0){
      return emegencyHandler();
    } else if (enermyOnSightFlag != 0){
      return dash();
    } else if (adrenalineFlag != 0){
      return adrenalineRush();
    } else if (scanFlag != 0){
      return scan();
    } else {
      return normalCycle();
    }

    decisionTimer = millis() + DECISION_INTERVAL;
  }
}

// replace prevMovingState and move.
void executeMove(){
  prevMovingState = movingState;
  basicMove(movingState, speed);
}

void resetSensorStats(){
  // LIGHT SENSOR
  lightSensorReadIndex = 0;
  lightSensorValueSum = 0;
  lightSensorValue = 1; // The sensor value meant to be used.

  // TOUCH SENSOR
  touchSensorReadIndex = 0;
  touchSensorValueSum = 0;
  touchSensorValue = 100; // The sensor value meant to be used.

  // DISTANCE SONAR SENSOR
  sonarReadIndex = 0;
  distanceSum = 0;
  distance = 1; // The sensor value meant to be used.
}

void resetFlags(){
  OUT_OF_RING_FLAG = 0;
  scanFlag = 0;
  breakerFlag = 0;
  enermyOnSightFlag = 0;
  adrenalineFlag = 0;
}

void resetTimers(){
  printTimer = millis() + 5;
  lightSensorPingTimer = millis() + 10;
  touchSensorPingTimer = millis() + 15;
  sonarPingTimer = millis() + 20;
  decisionTimer = millis() + 30;
  emegencyTimer = millis() + 35;
  breakerTimer = millis() + 40;
  moveTimer = millis() + 45;
  nomalCycleTimer = millis() + 50;

}

void resetSensorValues(){
  distance = 40;
  lightSensorValue = 1;
  touchSensorValue = 100;
}


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  resetSensorStats();
  resetTimers();
  resetFlags();
  
  // activate sensors for test
  refreshAllSensors();
  delay(100);
  refreshAllSensors();
  
  resetSensorValues();
  delay(100);
  

  // cruisingCount = 0;

}

void loop() {
  // Refresh Sensors
  refreshAllSensors();
  // Decision Making for setting up movingState
  int stateChanged = decisionMaking();
  
  // Change motar states if needed 
  if (stateChanged)
    executeMove();
  // wait 1 milliseconds before the next loop
  delay(1);
}

