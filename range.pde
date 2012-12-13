/* Ultrasound Sensor
 *------------------
 *
 * Reads values (00014-01199) from an ultrasound sensor (3m sensor)
 * and writes the values to the serialport.
 *
 * http://www.xlab.se | http://www.0j0.org
 * copyleft 2005 Mackie for XLAB | DojoDave for DojoCorp
 *
 */

#include <Servo.h> 

#define SWEEP_ANGLE_RANGE    170
#define SWING_PERIOD         600 // Time for one sweep from 0 to 180 degrees
#define ULTRA_SOUND_TIME_OUT 50000
#define HISTORY_SIZE         10
#define ZONE_COUNT           5

#define ZONE_LEFT            4
#define ZONE_FRONT_LEFT      3
#define ZONE_CENTRE          2
#define ZONE_FRONT_RIGHT     1
#define ZONE_RIGHT           0


int ultraSoundControl = 8; // Ultrasound signal pin
int ultraSoundSignal = 12; // Ultrasound signal pin
int ledPin = 11; // LED connected to digital pin 11
int sweepPin = 7, sweepFlip = 0, sweepDirection = 0, measureDirection = 0;
int steerPin = 13, steerLeft = 30, steerRight = 0, steerCentre = 12;
int startPin = 2;
unsigned long sweepStartTime = 0, usStartTime = 0, usMeasureStartTime = 0;
int motor[] = {5,6};
unsigned long zoneBanks[HISTORY_SIZE][ZONE_COUNT][2];
int currentBank = 0, measureBank = 0;

Servo sweep;  // create servo object to control the sensor servo
Servo steer;  // create servo object to control the steering servo

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);            // Sets the digital pin as output
  pinMode(ultraSoundControl, OUTPUT); // Switch control pin to output
  pinMode(ultraSoundSignal, INPUT);   // Switch signal pin to input
  pinMode(motor[0], OUTPUT);
  pinMode(motor[1], OUTPUT);
  pinMode(startPin, INPUT);
  digitalWrite(ultraSoundControl, LOW); // Send low pulse to ultrasound control
  
  sweep.attach(sweepPin);
  sweep.write(sweepDirection); // Reset ultrasound servo
  steer.attach(steerPin);
  steer.write(steerCentre); // Reset steering servo
  delay(SWING_PERIOD); // Wait for servo to come to rest position
  sweepStartTime = millis();
}

void loop() {
  // Motion stop switch
  while(digitalRead(startPin) == LOW){hold();}
  
  // Control sweep motion
  sweepSensor();
  
  // Control ultrasonic distance measurement
  checkUltraSound();

  // Make decision
  makeDecision(); 

}

void makeDecision(){
  if(measureBank == currentBank) return;
  
  unsigned long * left = zoneBanks[measureBank][ZONE_LEFT][1] == 0        ? zoneBanks[currentBank][ZONE_LEFT]        : zoneBanks[measureBank][ZONE_LEFT];
  unsigned long * frlf = zoneBanks[measureBank][ZONE_FRONT_LEFT][1] == 0  ? zoneBanks[currentBank][ZONE_FRONT_LEFT]  : zoneBanks[measureBank][ZONE_FRONT_LEFT];
  unsigned long * cetr = zoneBanks[measureBank][ZONE_CENTRE][1] == 0      ? zoneBanks[currentBank][ZONE_CENTRE]      : zoneBanks[measureBank][ZONE_CENTRE];
  unsigned long * frrt = zoneBanks[measureBank][ZONE_FRONT_RIGHT][1] == 0 ? zoneBanks[currentBank][ZONE_FRONT_RIGHT] : zoneBanks[measureBank][ZONE_FRONT_RIGHT];
  unsigned long * rigt = zoneBanks[measureBank][ZONE_RIGHT][1] == 0       ? zoneBanks[currentBank][ZONE_RIGHT]       : zoneBanks[measureBank][ZONE_RIGHT];
  
  if(cetr[0] < 200){  // Prevent Hitting wall
    turnCentre();
    backward();
  }else if(cetr[0] < 600 // When front and right are blocked, turn left
        && rigt[0] < 400
        && (frlf[1] > 600 || left[1] > 600)){
    turnLeft();
    forward();
  }else if(cetr[0] < 600 // When front and left are blocked, turn right
        && left[0] < 400
        && (frrt[1] > 600 || rigt[1] > 600)){
    turnRight();
    forward();
  }else if(cetr[0] < 300 // When all sides are blocked, reverse back and right (so it turns left by default)
        && rigt[0] < 200
        && left[0] < 200){
    turnRight();
    backward();
  }else if(rigt[0] < 200){
    turnLeft();
    forward();
  }else if(rigt[0] > 300){
    turnRight();
    forward();
  }else{
    turnCentre();
    forward();
  }
}

void checkUltraSound(){
  if(usStartTime == 0){ // Start of the first measurement
    triggerUltraSound();
    return;
  }else{
    int signal = digitalRead(ultraSoundSignal);
    if(usStartTime > usMeasureStartTime){ // If it is still waiting signal to go HIGH
      if(signal == HIGH){
        usMeasureStartTime = micros();
        return;
      }
    }else { // If it is waiting for the signal to go LOW
      if(signal == LOW){ // Calculate whole period that is high, and calculate distance
         unsigned long l = (micros() - usMeasureStartTime) * 17 / 100;;
         updateDistance(l);
         triggerUltraSound();
         return;
      }
    }
  }
  unsigned long t = micros() - usStartTime;
  if(t > ULTRA_SOUND_TIME_OUT){
    updateDistance(5000);
    triggerUltraSound();
  }
}

void updateDistance(unsigned long l){
  int zone = measureDirection * ZONE_COUNT / SWEEP_ANGLE_RANGE;
  if(zoneBanks[measureBank][zone][0] > l) zoneBanks[measureBank][zone][0] = l;
  if(zoneBanks[measureBank][zone][1] < l) zoneBanks[measureBank][zone][1] = l;
}

void triggerUltraSound(){
  digitalWrite(ultraSoundControl, HIGH); // Send high pulse
  delayMicroseconds(1);
  digitalWrite(ultraSoundControl, LOW); // Holdoff
  measureDirection = sweepDirection + (sweepFlip == 0 ? 0 : 20);
  usStartTime = micros();
}

void sweepSensor(){
  unsigned long t = millis() - sweepStartTime;
  if(t > SWING_PERIOD){
    // Change the sweep direction indicator
    sweepFlip = sweepFlip == 0 ? 1 : 0;
    
    // Reset the sweep timer
    sweepStartTime = millis();
    t = 0;
    currentBank = measureBank++;
    if(measureBank >= HISTORY_SIZE) measureBank -= HISTORY_SIZE;
    for(int i = 0; i < ZONE_COUNT; i ++){
      Serial.print("[");
      Serial.print(zoneBanks[currentBank][i][0]);
      Serial.print(",");
      Serial.print(zoneBanks[currentBank][i][1]);
      Serial.print("]");
      zoneBanks[measureBank][i][0] = 10000;
      zoneBanks[measureBank][i][1] = 0;
    }
    Serial.println("=");
  }
  if(sweepFlip == 0){
    sweepDirection = t * SWEEP_ANGLE_RANGE / SWING_PERIOD;
  }else {
    sweepDirection = (SWING_PERIOD - t) * SWEEP_ANGLE_RANGE / SWING_PERIOD;
  }
  //Serial.println(sweepDirection);
  sweep.write(sweepDirection);
}

void turnLeft(){
  steer.write(steerLeft);
}

void turnRight(){
  steer.write(steerRight);
}

void turnCentre(){
  steer.write(steerCentre);
}

void forward(){
//  digitalWrite(motor[0],HIGH);
  analogWrite(motor[0],210);
  digitalWrite(motor[1],LOW);
}

void hold(){
  digitalWrite(motor[0],LOW);
  digitalWrite(motor[1],LOW);
}

void backward(){
  digitalWrite(motor[0],LOW);
//  digitalWrite(motor[1],HIGH);
  analogWrite(motor[1],210);
}
