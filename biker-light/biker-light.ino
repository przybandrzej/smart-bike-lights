#include "LedControl.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

LedControl lc = LedControl(12, 11, 10, 1);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double xError = 0;
double yError = -9.81;
double zError = 1.0;
double sensorResolution = 0;
int switch_pin = 7;

enum Side {
  LEFT, RIGHT
};

bool isOn = false;
int loopIterator = 0;
bool light = false;
bool value = false;
Side sideOn = Side.LEFT;

void setup() {
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
  Serial.begin(9600);
  accel.begin()
  accel.setRange(ADXL345_RANGE_2_G);
  sensor_t sensor;
  accel.getSensor(&sensor);
  sensorResolution = sensor.resolution;
  pinMode(switch_pin, INPUT);
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  double acc = event.acceleration.x;
  //Serial.print("Read X "); Serial.print(acc); Serial.println("m/s^2 ");
  acc = acc - xError;
  if (acc < 0) {
    acc = acc + sensorResolution;
  } else if (acc > 0 ) {
    acc = acc - sensorResolution;
  }
  /*Serial.print("Sensor resolution "); Serial.println(sensorResolution);
  Serial.print("X: "); Serial.print(acc); Serial.print("  "); Serial.println("m/s^2 ");
  double vx = velocity0 + acc * 0.25;
  Serial.print("X velocity: "); Serial.print(vx);  Serial.println("m/s "); Serial.println("");*/

  if (loopIterator == 8) {
    loopIterator = 0;
    light = false;
    Serial.print("Turn "); Serial.print(sideOn); Serial.println(" light off.");
  }
  value = digitalRead(switch_pin); // check mercury switch state
  if (light) {
    if (isOn) {
      turnOffTurnLight(sideOn);
      isOn = false;
    } else {
      turnLight(sideOn);
      isOn = true;
    }
  } else {
    if (value) {
      if (acc < 0) {
        sideOn = RIGHT;
      } else {
        sideOn = LEFT;
      }
      light = true;
      loopIterator = -1;
      Serial.print("Turn "); Serial.print(sideOn); Serial.println(" light on.");
    }
    turnOffTurnLight(LEFT);
    turnOffTurnLight(RIGHT);
  }
  loopIterator++;
  delay(250);
}

void turnLight(Side side) {
  int beginCol = 0;
  int endCol = 8;
  if (side == LEFT) {
    endCol = 4;
  }
  if (side == RIGHT) {
    beginCol = 4;
  }
  for (int row = 0; row < 8; row++) {
    for (int col = beginCol; col < endCol; col++) {
      lc.setLed(0, col, row, true);
    }
  }
}

void turnOffTurnLight(Side side) {
  int beginCol = 0;
  int endCol = 8;
  if (side == LEFT) {
    endCol = 4;
  }
  if (side == RIGHT) {
    beginCol = 4;
  }
  for (int row = 0; row < 8; row++) {
    for (int col = beginCol; col < endCol; col++) {
      lc.setLed(0, col, row, false);
    }
  }
}
