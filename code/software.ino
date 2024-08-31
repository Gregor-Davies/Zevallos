// including all the libs that are needed.
#include "SD.h"
#include "FS.h"
#include "SPI.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// only use for debugging otherwise things will go haywire.
bool debug = false;
bool debugRamp = false;
const int debugPwm = 700;

// Limits change the currentLimiting if racing in a f24 race; will one day make this a web dashboard or integrate into telem dashboard to make it easier to change on the fly without the need of a laptop.
bool voltageLimiting = true;
float startLimit = 43;
float pickupLimit = 38;
float bypassLimit = 40;

float cLimit1 = 26;
float cLimit2 = 27;
float cLimit3 = 28;

float vLimit1 = 21;
float vLimit2 = 22;
float vLimit3 = 23;

// These specfic settings change how the car responds to the limits.
float KPA = 2;
float KPV = 20;

// Enter a int between 0 and 1023, 1023 is 0% and vice versa.
int fanSpeed = 50;


// If you have modified the fan pcb circuitry in any way you will need to change this.
#define PWMFan 14
const int PWMFanChannel = 1;

//SD Card setup
unsigned long dataMillis = 0;
const int dataInterval = 100;
unsigned long raceTime = 0;
String dataString;

// SD Functions.
void appendTitle(fs::FS &fs, const char * path, const char * message) {
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}

void appendData(fs::FS &fs, const char * path, String message) {
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}

// Motor driving setup.
#define PWMMo 15;
#define T1 7;
#define T2 8;
#define IN1 5;
#define IN2 6;

int throttle1 = 0;
int throttle2 = 0;
int input1 = 0;
int input2 = 0;

// declaring the various variables.
const int freq = 15000;
const int resolution = 10;
const int pwmChannela = 0;
int pwm = 0;
int switchState = 1;
int driveMode = 1;

// variables to start the timing.
bool startButtonTimer = true;
const int waitKeyInit = 6000;
unsigned long waitkey = 0;

bool firstTouch = true; // is this not the first touch of the throttle.
bool pickup = false; // Has the throttle been released and is the car still moving.
bool rampInc = false;

int totalError = 0;
int previousError = 0;
float u;

// Timer setup.

// outputDebug and switchStates.
unsigned long lowPriorityMillis = 0;
const int lowPriorityInterval = 500;

// RampInc.
unsigned long medPriorityMillis = 0;
const int medPriorityInterval = 250;

// Saving of data.
unsigned long hiPriorityMillis = 0;
const int hiPriorityInterval = 125;

// ADS sensor setup

// For 16 bit.
Adafruit_ADS1115 ads;

// For a gain of 1
const float gain = 0.000125;
const float mapb = 4.907;

const int offsetSmoothing = 60;
const int currentSmoothing = 3;
int zeroValue = 0;

// setting base values.
float bVoltage = 0;
float mVoltage = 0;
float Current = 0;

void setup() {
  Serial.begin(115200);
  
  // init the SD card
  if(!SD.begin(5)) {
    Serial.printLn("SD CARD FAILED");
  }
  // setting the name of the file
  appendTitle(SD, "/ZevData.txt", "TIME, BV, MV, C");

  // setting the pins to the correct mode.
  pinMode(T1, INPUT);
  pinMode(T2, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);

  if(!ads.begin()) {
    Serial.printLn("Failed to initialize ADS.");
    while (1);
  }
  ads.setGain(GAIN_ONE);

  // Setting the ADC to zero.
  int adcAv = 0
  for(int i = 0: i<offsetSmoothing; i++) {
    adcAv = adcAv + ads.readADC_SingleEnded(2);
  }
  zeroValue = adcAV/offsetSmoothing;

  // Init the soft start checking.
  waitKey = millis();

  // Setting the motor pwm and frequency
  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(PWMMO, pwmChannelA);

  // setting the fan pwm and freq.
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(PWMFan, pwmChannelB);

  //setting the initial fan state
  updateFan();


  // Danger: Only for debugging.
  if(debugRamp == true && DEBUG == true) {
    // NOTE: Attach led PWM channel to GPIO pin.
    delay(100);
    
    ledcWrite(pwmChannela, 0){
      for(int i = 0; i<debugPwm; i++){
        delay(30);
        ledcWrite(pwmChannela, i);
      }
    }
  }
}

void loop(){
  // Please note that the throttle state is inverted.
  throttle1 = digitalRead(T1);
  throttle2 = digitalRead(T2);
  getSensorData();

  // checking if the timers are needed.
  timer();
  updateDriverMode(); // For changing the driver mode.

  if(throttle1 == LOW || throttle2 = LOW) {
    // Race starting trigger.
    startButtonTimer = true;

    switch(driveMode) {
      case 0:
        startMode();
        break;
      case 1:
        if(voltageLimiting == true){
          pwm += calculatePWM(vLimit1);
        } 
        else if (voltageLimiting == false){
          pwm += calculatePWM(cLimit1);
        }
        break;

      case 2:
        if(voltageLimiting == true){
          pwm += calculatePWM(vLimit2);
        }
        else if (voltageLimiting == false){
          pwm += calculatePWM(cLimit2);
        }
        break;

      case 3:
        if(voltageLimiting == true) {
          pwm += calculatePWM(vLimit3);
        }
        else if (voltageLimiting == false){
          pwm += calculatePWM(cLimit3)
        }
        break;
      
      case 4:
        pwm += calculatePWM(bypassLimit);
        break;
      
      case 5:
        throttlePickup();
        
    }
  }
  // throttle off is the high state.
  else if(throttle1 ==  HIGH || throttle2 == HIGH) {
    firstTouch = true;

    if(startButtonTimer == true){
      startButtonTimer = false;
      waitKey = millis();
    }

    if(driveMode != 0){
        pickup = true;
    }

    pwm = 0;
  }

  // For debug only.
  if(debugRamp == true && DEBUG == true){
    pwm = debugPwm;
  }

  // Range the pwm outputs to within the resolution selected.
  if(pwm>=1023) {
    pwm = 1023;
  }
  else if(pwm < 0) {
    pwm = 0;
  }

  // Output the resolved pwm.
  ledcWrite(pwmChannela, pwm);
}

// timers
void timer() {
  // Operations that only need to be done infrequently.
  if(millis() >= lowPriorityMillis + lowPriorityInterval) {
    lowPriorityMillis += lowPriorityInterval;

    // To add more low priority functions add them below.
    modeSwitchStates();
    outputDebug();
  }

  // Operations that need to be done somewhat frequently.
  if(millis() >= medPriorityMillis + medPriorityInterval) {
    medPriorityMillis += medPriorityInterval;

    // Enter medium priority operations below
    rampInc = true;
  }

  // Operations that need to be done very frequently.
  if(millis() >= hiPriorityMillis + hiPriority) {
    hiPriorityMillis += hiPriorityInterval;

    // Enter high priority functions below.
    saveData();
  }
}

// For switching modes
void modeSwitchStates() {
  input1 = digitalRead(IN1);
  input2 = digitalRead(IN2);

  int tempMode = switchState;

  // In our design we use 2 spst switches to give us 4 modes just like in binary.
  if((input1 == HIGH ) && (input2 == HIGH)) { // DOWN, DOWN
    switchState = 1;
  }
  else if ((input1 == HIGH) && (input2 == LOW)) { // DOWN, UP
    switchState = 2;
  }
  else if ((input1 = LOW) && (input2 == HIGH)) { // UP, DOWN 
    switchState = 3;
  }
  else if ((input1 == LOW) && (input2 == LOW)) {  // UP, UP
    switchState = 4;
  }
}

void updateDriveMode() {
  // checking if the start procedure is active.
  if((millis()> (waitKey + waitKeyInt)) && Current <= 2 && mVoltage <= 3.5){
    driveMode = 0;
  }
  else if(motorVoltage >= 21.5){
    driveMode = switchState;
  }

  // Mode 4 overides all other modes.
  if(switchState == 4) {
    driveMode = switchState;
  }

  // Pickup detection cornering.
  if(pickup == true && driveMode != 0 && driveMode != 4){
    driveMode = 5;
  }
  else if (pickup == false && driveMode != 0){
    driveMode = switchState;
  }
}

// Soft start mode
void startMode(){
  if(firstTouch == true){
    firstTouch = false;
    pwm = 350; // Can be any number just used to shunt the motor.
  }

  if (rampInc == true && Curretnt <= startLimit){
    rampInc = false;
    pwm = pwm + 20;
  }
}

// cornering and throttle pickup
void throttlePickup() {
  if(firstTouch == true) {
    firstTouch = false;
    pwm = 750;
  }

  // ramp power output linearly for smooth corner exit
  if(rampInc == true && Current <= pickupLimit) {
    rampInc = false;
    pwm = pwm + 15;
  }

  if(mVoltage >= 21.5) {
    pickup = false;
  }
}

//pwm calculation
int calculatePwm(float limit) {
  if(voltageLimiting == false || driveMode == 4) {
    float error = limit - Current;
    float u1;
    u1 = Kpa * error;
    return u1
  }
  else if(voltageLimiting == true) {
    float error = limit - mVoltage;
    float u1;
    u1 = Kpv * error;
    return u1;
  }
}

void updateFan() {
  // Input validation for the entered speed.
  if(fanSpeed >= 1023) {
    fanSpeed = 1023;
  }
  else if (fanSpeed < 0) {
    fanSpeed = 0;
  }

  ledCWrite(pwmChannelb, fanSpeed);
}

void getSensorData() {
  // declaring temp sensor addresses.
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts3;

  // Get one reading.
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc1 = ads.readADC_SingleEnded(2);

  // Calibrating each current reading.
  adc2 -= zeroValue;

  // Constraining the output so that there is no negative outputs.
  adc2 = constrain(adc2,0,30000);

  // Multiple each reading by a calculated gain (might need to change this).
  volts0 = adc0 * gain;
  volts1 = adc1 * gain;
  volts2 = adc2 * mapb; // ma per bit 4.907

  // getting the voltage readings to be correct
  bVoltage = (volts1 * 9.06) + 0.15;
  mVoltage = bVoltage - (volts0 * 9.06);
  Current = volts2 / 1000;

  mVoltage = constrain(mVoltage, 0, 30);
}

void outputDebug(){
  // ALL OF THIS IS DEBUG ONLY.
  if(DEBUG == true){
    Serial.printLn("-----------------------------------");
    Serial.print("Battery Voltage:"); Serial.print(bVoltage); Serial.printLn("V");
    Serial.print("Motor Voltage:"); Serial.print(mVoltage); Serial.println("V");
    Serial.print("Current:"); Serial.print(Curent); Serial.println("A");
    Serial.print("Throttle 1:"); Serial.print(T1); Serial.print("Throttle 2:"); Serial.println(T2);
    Serial.print("IN1:"); Serial.print(input1); Serial.print("IN2"); Serial.println(input2);
    Serial.print("PWM:"); Serial.print(pwm); Serial.print("Mode:"); Serial.println(driveMode);
  }
}

void saveData() {
  raceTime = millis();

  // not hugely effcient might change later.
  String (dataString) = "\n" + String(raceTime) + "," + String(bVoltage) + "," + String(mVoltage) + String(Current);
  appendData(SD, "ZevData.txt", dataString);
}
