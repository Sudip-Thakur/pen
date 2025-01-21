#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int motorPin = 32;    // Haptic motor pin
int buttonPin = 15;   // Push button pin

// PWM configurations
int freq = 5000;
int ledChannel = 0;   
int resolution = 8;   
int dutyCycle = 200; 

bool readData = false; 

const float alpha = 0.2;

// Filtered values for acceleration and gyroscope
float filteredAx = 0, filteredAy = 0, filteredAz = 0;
float filteredGx = 0, filteredGy = 0, filteredGz = 0;

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(motorPin, ledChannel);

  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection())
  {
    Serial.println("MPU6050 connected successfully.");
  }
  else
  {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
}

void vibrateMotor(int duration)
{
  ledcWrite(ledChannel, dutyCycle); 
  delay(duration);
  ledcWrite(ledChannel, 0);
}

void readMPU6050Data(int duration)
{
  unsigned long startTime = millis();
  while (millis() - startTime < duration)
  {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Apply low-pass filter
    filteredAx = alpha * ax + (1 - alpha) * filteredAx;
    filteredAy = alpha * ay + (1 - alpha) * filteredAy;
    filteredAz = alpha * az + (1 - alpha) * filteredAz;
    filteredGx = alpha * gx + (1 - alpha) * filteredGx;
    filteredGy = alpha * gy + (1 - alpha) * filteredGy;
    filteredGz = alpha * gz + (1 - alpha) * filteredGz;

    // json data all in single line of code 
    // Serial.printf("{\"AccelX\":%d,\"AccelY\":%d,\"AccelZ\":%d,\"GyroX\":%d,\"GyroY\":%d,\"GyroZ\":%d}\n", ax, ay, az, gx, gy, gz); 

    // Print filtered data
    Serial.print(">AccelX: ");
    Serial.println(filteredAx);
    Serial.print(">AccelY: ");
    Serial.println(filteredAy);
    Serial.print(">AccelZ: ");
    Serial.println(filteredAz);
    Serial.print(">GyroX: ");
    Serial.println(filteredGx);
    Serial.print(">GyroY: ");
    Serial.println(filteredGy);
    Serial.print(">GyroZ: ");
    Serial.println(filteredGz);

  }
}

void loop()
{
  int buttonState = digitalRead(buttonPin);
  // if (buttonState == LOW) {
  //   vibrateMotor(200);
  // }
  if (buttonState == LOW && !readData)
  {
    readData = true; 
    vibrateMotor(200);
    readMPU6050Data(10000);
    vibrateMotor(200);
    readData = false;
  }
}
