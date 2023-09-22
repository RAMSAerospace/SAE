#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Wire.h>

#define GPSSerial Serial1
#define GPSECHO false
#define IMUECHO true

uint32_t timer1 = millis();
uint32_t timer2 = millis();

Adafruit_MPU6050 mpu;
Adafruit_GPS GPS(&GPSSerial);

void setup(){
    Serial.begin(115200);

    Serial.println("Init MPU");
    Wire.begin();
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
}

void setup1(){
  Serial.println("Init GPS");
  GPS.begin(9600);
}

void loop(){
  if (millis() - timer1 > 50) {
      timer1 = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (IMUECHO){
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    }
  }
}

void loop1(){
    char c = GPS.read();
    if (GPSECHO){
      if (c) Serial.print(c);
    }
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))
        return;
    }

    if (millis() - timer2 > 1000) {
      timer2 = millis();
      Serial.println("");
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" Quality: "); Serial.print((int)GPS.fixquality);
      Serial.print(" Satellites: "); Serial.println((int)GPS.satellites);
      if (GPS.fix) {
        Serial.print("Location: "); 
        Serial.print(GPS.latitude, 6); Serial.print(GPS.lat); Serial.print(", ");
        Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.println("");
      }
    }

    
}