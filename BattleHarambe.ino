/*
  Battle Harambe Ver.1.8

 created 20 Nov. 2016
 modified 21 Nov 2016
 by Phoenix

 */

 #include <NewPing.h>

 /*
  * ======================
  * Fields Declarations 
  * ======================
 */
// LIGHT SENSOR
const int LIGHT_SENSOR_PIN = A4;  // Analog input pin that the LIGHT_SENSOR is attached to
// TOUCH SENSOR
const int TOUCH_SENSOR_PIN = A0;  // Analog input pin that the TOUCH_SENSOR is attached to
// DISTANCE SENSOR
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;
const int MAX_DISTANCE = 200;
const int PING_INTERVAL = 20;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sonar object
// MOTARS
const int MOTOR_A1 = 10;
const int MOTOR_A2 = 9;

const int MOTOR_B1 = 6;
const int MOTOR_B2 = 5;

// Shared value vars
int lightSensorValue = 1;
int touchSensorValue = 10;
unsigned int distance = 40;
unsigned long pingTimer = 0;

// FLAGS
int adrenalineFlag = 0; // Flag to trigger adrenalineRush attack [10 for initialize an attack]
int enermyOnSightFlag = 0; // Flag if enermy detected in sight
int scanFlag = 0; // Flag to perform a scan
int OUT_OF_RING_FLAG = 0;// Critical Flag to show out of RING [3 for initialize an emegency move]

// Count for cruising performed;
int cruisingCount = 0;
int MAX_CRUISING_COUNTS = 3;

 /*
  * ======================
  * Sensor Reading Functions 
  * ======================
 */

 
// Average readings from the sensor
int avgReadingAnalog(int pin, int samplesize){
  int sum = 0;
  for(int i = 0; i<samplesize; i++){
    sum += analogRead(pin);
    delay(2);
  }
  return (sum / samplesize);
}

int lightCumulator(int pin, int totalTimes){
  for(int i =0; i<totalTimes; i++){
    if (analogRead(pin) != 2){
      delay(2);
      if (analogRead(pin) != 2){
        return 1;
      }
    }
    delay(1);
  }
  return 2;
}

int refreashLightSensorOnly(){
  lightSensorValue = avgReadingAnalog(LIGHT_SENSOR_PIN, 10);
  if (lightSensorValue > 1){
    OUT_OF_RING_FLAG = 3;
    return 3;
  }
  return 0;
}

// Sonar reading helper function
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar.check_timer())
    distance = sonar.ping_result / US_ROUNDTRIP_CM;
}

// refreash Sonar sensor
void readSonarSensor(){
  if (millis() >= pingTimer) {         // Is it this sensor's time to ping?
      pingTimer += PING_INTERVAL;  // Set next time this sensor will be pinged.
//      oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar.timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
//      distance = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar.ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
}

// Refresh all sensor stats and set flags if needed.
void refreshSensors(){
  
  if (refreashLightSensorOnly())
    return;
  readSonarSensor();
  
  touchSensorValue = avgReadingAnalog(TOUCH_SENSOR_PIN, 2);
  if (touchSensorValue == 0 && adrenalineFlag == 0){
    adrenalineFlag = 5;
  }
  else if (distance < 35){
    adrenalineFlag = 0;
    enermyOnSightFlag = 1;
  } else {
    enermyOnSightFlag = 0;
  }

//  printSensorStats(); // May increase performance if commented.
}



// print the results to the serial monitor:
void printSensorStats(){
  Serial.print("Light sensor = ");
  Serial.print(lightSensorValue);
  Serial.print("\t Touch sensor = ");
  Serial.print(touchSensorValue);
  Serial.print("\t Sonar sensor = ");
  Serial.println(distance);

}

 /*
  * ======================
  * Basic Movement Functions 
  * ======================
 */
void stop() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
}

void moveForward(int spd) {
  analogWrite(MOTOR_A1, spd);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, spd);
}

void moveBackward(int spd) {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, spd);
  analogWrite(MOTOR_B1, spd);
  analogWrite(MOTOR_B2, 0);
}

void turnLeft(int spd) {
  analogWrite(MOTOR_A1, spd);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, spd);
  analogWrite(MOTOR_B2, 0);
}
void turnRight(int spd) {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, spd);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, spd);
}

// Main move function, should be used in almost any case.
void intervalMove(void (*f)(int), int spd, int intervals){
  int DELAY_TIME = 5;
  int turning = 0;
  if (f == turnLeft || f == turnRight)
    turning = 1;
  stop();
  delay(5);
  for(int i = 0; i < intervals; i++){
    refreshSensors();
    delay(2);
    if (OUT_OF_RING_FLAG != 0)
      break;
    f(spd);
    delay(DELAY_TIME);
    if (turning){
      delay(10);
      stop();
      delay(5);
    }
  }
  delay(5);
}

// Special move function that does NOT CHECK events 
void unstoppableMove(void (*f)(int), int spd, int intervals){
  int DELAY_TIME = 5;
  int turning = 0;
  if (f == turnLeft || f == turnRight)
    turning = 1;
  stop();
  delay(5);
  for(int i = 0; i < intervals; i++){
    f(spd);
    delay(DELAY_TIME);
    if (turning){
      delay(10);
      stop();
      delay(5);
    }
  }
  delay(3);
}

 /*
  * ======================
  * Strategic Functions 
  * ======================
 */
// Normal moves.
void normalCycle(){
  intervalMove(moveForward, 180, 2);
//  intervalMove(turnLeft, 255, 5);
}

// Dash forward!
void dash(){
  int duration = 10;
  int DELAY_TIME = 2;
  stop();
  delay(3);

  while(duration-- > 0){
    if (refreashLightSensorOnly()){
      stop();
      return;
    }
    moveForward(255);
    delay(5);
  }
}

// Brief Scan for enermy
void scan(){
  scanFlag = 0; // Reset scan flag
  int intervals = 50;
  while(intervals-- > 0){
    unstoppableMove(turnLeft, 255, 2); // Turn Left
    readSonarSensor();// Refreash Sensor
    if (enermyOnSightFlag != 0)
      return;
  }
  while(intervals++ < 20){
    unstoppableMove(turnRight, 255, 1); // Turn Left
  }
}

// adrenalineRush attack! jump forward and make an 180degree turn.
void adrenalineRush(){
  switch(adrenalineFlag--){
    case 5:
    case 4:
      dash();
      dash();
      break;
    case 3:
      unstoppableMove(turnRight, 255, 80);
      break;
    case 2:
      enermyOnSightFlag = 0;  
    default:
      adrenalineFlag = 0;
  }
}

// emergencyMove function, handles the actual move (move backwards, and turn left a bit)
void emergenceMove(){
  switch(OUT_OF_RING_FLAG--){
    case 3:
        // check if a scan should be performed soon after;
        if (++cruisingCount > MAX_CRUISING_COUNTS){
          scanFlag = 1;
          cruisingCount = 0;
        }
      unstoppableMove(moveBackward, 255, 80); // Move backwards
      break;
    case 2:
      unstoppableMove(turnLeft, 255, 40); // Turn Left
      break;
    default:
      OUT_OF_RING_FLAG = 0; // Back to normal
  }
}

// Handle events
void eventHandler(){
  if (OUT_OF_RING_FLAG != 0){
    adrenalineFlag = 0;
    emergenceMove();
  }
  else if(adrenalineFlag != 0){
    adrenalineRush();
  }
  else if(enermyOnSightFlag != 0){
    scanFlag = 0;
    dash();
   }else if(scanFlag != 0){
     scan();
  }
  else{
    normalCycle();
  }
}


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pingTimer = millis() + 75;

  adrenalineFlag = 0; // Flag to trigger adrenalineRush attack [10 for initialize an attack]
  enermyOnSightFlag = 0; // Flag if enermy detected in sight
  scanFlag = 0; // Flag to perform a scan
  OUT_OF_RING_FLAG = 0;// Critical Flag to show out of RING [3 for initialize an emegency move]
  cruisingCount = 0;

  delay(500);
  refreshSensors();
  delay(1000); //Delay for sensors to be initialized;
  refreshSensors();
  delay(500);

  //sensor value initialize
  distance = 50;
  lightSensorValue = 1;
  touchSensorValue = 100;
}

void loop() {
  delay(2);  
  refreshSensors();
  eventHandler();
  // wait 2 milliseconds before the next loop
  delay(2);
}


