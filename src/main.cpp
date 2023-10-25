// ------------ PACKAGES ------------ //

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// IMU
// #include <DFRobot_BMX160.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;
// DFRobot_BMX160 bmx160;

// Radio
#include <RF24.h>
#include <nRF24L01.h>
RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

// ESC
#include <Servo.h>
Servo backESC, frontESC, rightESC, leftESC;

// ------------ STRUCTURES ------------ //

struct monitorPack
{
  int id = 1;
  int forceShutdown = 0;
  float thrust = 0.0;
  float pitch = 0.0;
  float roll = 0.0;
  float v_curr = 0.0;
  float v_bat_1 = 0.0;
  float v_bat_2 = 0.0;
  float v_bat_3 = 0.0;
};
monitorPack monitorData;

struct controlPack
{
  int id = 0;
  int isActive = 0;
  float thrust = 0.0;
};
controlPack controlData;

// float k1 = 0.5, k2 = 250, k3 = 0.001;
// float rollActual;
// float pitchTarget = 0.0;
// float pitchActual;
// float pitchError = 0.0;
// float pitchCorrP = 0.0;
// float pitchCorrI = 0.0;
// float yawActual;

// ------------ PINS ------------ //

// ESC
int const rightESCpin = 8;
int const leftESCpin = 7;
int const frontESCpin = 6;
int const backESCpin = 5;

// Battery
int vBatPin1 = A0;
int vBatPin2 = A1;
int vBatPin3 = A2;
int cBatPin = A6;

// ------------ VARIABLES ------------ //

// Time
// int milliOld;
// int milliNew;
float millisOld = 0.0;
float dt;

// Read IMU Data
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;
float pitchAcc, rollAcc;
float pitch, roll;

// Correct for Radio Errors
int isActive = 0;
int prevIsActive = 0;
int forceShutdown = 0;
// int consistancy_count = 0;

// Calculate Errors
float pitchError = 0.0;
float pitchTarget = 0.0;
float pitchCorrP;
float pitchCorrI = 0.0;
float pitchCorrD;
float pitchErrorOld = 0.0;

// Run Motors
float thrust = 0;
float frontSig = 0.0, backSig = 0.0, rightSig = 0.0, leftSig = 0.0;

// Read Battery
float vRef, vBat1, vBat2, vBat3;
float currBat, newCurrBat, lastCurrBat = 0;

// Run Coms
float interval = 1000000.0; // 100000.0;
float lastTime = 0.0;
int isComsInterrupted = 0;

// ------------ SETUP ------------ //

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // Initialize ESCs
  frontESC.attach(frontESCpin, 1000, 2000);
  backESC.attach(backESCpin, 1000, 2000);
  leftESC.attach(leftESCpin, 1000, 2000);
  rightESC.attach(rightESCpin, 1000, 2000);

  // Initialize Radio
  delay(100);
  radio.begin();
  radio.openWritingPipe(addresses[1]);    // 00001
  radio.openReadingPipe(1, addresses[0]); // 00002
  radio.setDataRate(RF24_2MBPS);          // Set the speed of the transmission to the quickest available
  radio.setChannel(124);                  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  radio.setPALevel(RF24_PA_MAX);          // Set radio Tx power to MAX
  radio.startListening();
  // Serial.println(radio.isChipConnected());

  delay(100);

  // Battery
  pinMode(vBatPin1, INPUT);
  pinMode(vBatPin2, INPUT);
  pinMode(vBatPin3, INPUT);
  pinMode(cBatPin, INPUT);

  // Initialize IMU
  if (!mpu.begin())
  {
    // Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // LEDs
  pinMode(2, OUTPUT);

  Serial.println("setup");
}

// ------------ FUNCTIONS ------------ //

void readIMU()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro_x = g.gyro.x + 0.05;
  gyro_y = g.gyro.y - 0.02;
  gyro_z = g.gyro.z;

  accel_x = a.acceleration.x - 0.11;
  accel_y = a.acceleration.y - 0.26;
  accel_z = a.acceleration.z - 0.3;
}

void calcEulerAngles()
{

  pitchAcc = -atan2(accel_y / 9.8, accel_z / 9.8) / 2 / 3.141592654 * 360;
  rollAcc = atan2(accel_x / 9.8, accel_z / 9.8) / 2 / 3.141592654 * 360;

  // pitchGyr = pitchGyr + gyro_x * dt / 2 / 3.141592654 * 360;
  // rollGyr = rollGyr + gyro_y * dt / 2 / 3.141592654 * 360;

  pitch = 0.95 * (pitch - gyro_x * dt / 2 / 3.141592654 * 360) + 0.05 * pitchAcc;
  // pitch = -pitch;
  roll = 0.95 * (roll - gyro_y * dt / 2 / 3.141592654 * 360) + 0.05 * rollAcc;
  // roll = -roll;
}

void readBattery()
{
  vBat1 = analogRead(vBatPin1);
  vBat1 = (vBat1 / 1023) * 5; // * 5.28;
  vBat1 = vBat1 * 1.0025706941;

  vBat2 = analogRead(vBatPin2);
  vBat2 = (vBat2 / 1023) * 5; // 5.28;

  vBat3 = analogRead(vBatPin3);
  vBat3 = (vBat3 / 1023) * 5; // 5.28;
  vBat3 = vBat3 * 1.0478723404;

  newCurrBat = analogRead(cBatPin);
  newCurrBat = newCurrBat * 5 / 1023.0;
  newCurrBat = (newCurrBat - 2.5) / 0.066 + 0.2;

  currBat = newCurrBat * 0.1 + lastCurrBat * 0.9;
  lastCurrBat = currBat;
}

void calculateErrors()
{
  pitchError = pitchTarget - pitch;

  pitchCorrP = pitchError * 0.08; // 0.04
  // pitchCorrP = pitchError * 0.07; // 0.03
  pitchCorrI = pitchCorrI + pitchError * dt * 0;           // 0.01;
  pitchCorrD = ((pitchError - pitchErrorOld) / dt) * 0.02; // 0.02
  pitchErrorOld = pitchError;
}

void calcFlightMode()
{
  // Handle Shutdown
  if (vBat1 < 2.7 || vBat2 < 2.7 || vBat3 < 2.7 // Battery too low
      || currBat > 30                           // Current too high
      || isComsInterrupted == 1                 // Coms failed
      || pitch > 45 || pitch < -45              // Drone flipped
      || roll > 45 || roll < -45                // Drone flipped
  )
  {
    forceShutdown = 1;
  }
  else
  {
    forceShutdown = 0;
  }

  // Handle Mode
  if (forceShutdown == 1)
  {
    isActive = 0;
  }
  else
  {
    isActive = controlData.isActive;
  }
}

void calcThrust()
{
  if (isActive == 0)
  {
    thrust = 0;
    pitchCorrP = 0;
    pitchCorrI = 0;
    pitchCorrD = 0;
  }
  else
  {
    thrust = map(controlData.thrust, 0, 100, 0, 180);
  }
}

void runLEDs()
{
  if (isActive == 1)
  {
    digitalWrite(2, HIGH);
  }
  else
  {
    digitalWrite(2, LOW);
  }
}

void runMotors()
{
  if (isActive == 1)
  {
    frontSig = thrust + pitchCorrP + pitchCorrD + pitchCorrI;
    backSig = thrust - pitchCorrP - pitchCorrD - pitchCorrI;
    leftSig = 0;
    rightSig = 0;
  }
  else
  {
    frontSig = 0;
    backSig = 0;
    leftSig = 0;
    rightSig = 0;
  }

  // Write servo signal
  backESC.write(backSig);
  frontESC.write(frontSig);
  rightESC.write(rightSig);
  leftESC.write(leftSig);
}

void populateMonitorData()
{
  monitorData.thrust = controlData.thrust;
  monitorData.forceShutdown = forceShutdown;
  monitorData.pitch = pitch;
  monitorData.roll = roll;
  monitorData.v_curr = -currBat;
  monitorData.v_bat_1 = vBat1;
  monitorData.v_bat_2 = vBat2;
  monitorData.v_bat_3 = vBat3;
  monitorData.id++;
}

void runComs()
{
  // SEND DATA
  radio.stopListening();

  if (!radio.write(&monitorData, sizeof(monitorData)))
  {
    // Serial.println("❌ send failed...");
    isComsInterrupted = 1;
  }
  else
  {
    isComsInterrupted = 0;
    //  Serial.println("✅ sent...");
  }

  // delay(5);
  // RECEIVE DATA
  radio.startListening();

  unsigned long start_receiving_at = millis();
  boolean timeout = false;

  while (!radio.available())
  {
    if (millis() - start_receiving_at > 200)
    {
      timeout = true;
      return;
    }
  }
  if (timeout)
  {
    // Serial.println("❌ receive timeout...");
    isComsInterrupted = 1;
  }
  else
  {
    radio.read(&controlData, sizeof(controlData));
    //  Serial.println("✅ received...");
    isComsInterrupted = 0;
  }
}

// ------------ LOOP ------------ //

float i = 0.0;
void loop()
{
  // dt = (micros() - millisOld) / 1000000.0;
  // millisOld = micros();

  // GET
  readIMU();

  // readBattery();

  // CALCULATE
  // calcFlightMode();
  // calculateErrors();
  // calcThrust();
  // calcEulerAngles();

  // WRITE
  // runLEDs();
  // runMotors();

  // SHARE
  // populateMonitorData();
  // if (micros() > lastTime + interval)
  //{
  //  runComs();
  //   lastTime = micros();
  //}

  i = i + 0.1;
  Serial.print(i * 10);
  Serial.print(",");
  Serial.println(micros());

  // Serial.print("loop");
  // Serial.println(i);
  //
  // Serial.print(">Pitch Corr P:");
  // Serial.println(pitchCorrP);
  //
  // Serial.print(">Pitch Corr I:");
  // Serial.println(pitchCorrI);
  //
  // Serial.print(">Pitch Corr D:");
  // Serial.println(pitchCorrD);
}