#include "LedControl.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/*
 * Pin 12 is connected to the DATA IN-pin of the first MAX7221
 * Pin 11 is connected to the CLK-pin of the first MAX7221
 * Pin 10 is connected to the LOAD(/CS)-pin of the first MAX7221 
 * 1 MAX7221 attached to the arduino
 */
LedControl lc = LedControl(12, 11, 10, 1);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

double xError = 0;
double sensorResolution = 0;
int switch_pin = 7;

enum Side {
  LEFT, RIGHT
};

bool isOn = false;
int loopIterator = 0;
bool light = false;
int val = LOW;
Side sideOn = LEFT;

void setup() {
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
  Serial.begin(9600);
  accel.begin();
  accel.setRange(ADXL345_RANGE_2_G);
  sensor_t sensor;
  accel.getSensor(&sensor);
  sensorResolution = sensor.resolution;
  pinMode(switch_pin, INPUT);
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);
  double acc = event.acceleration.x;
  if (acc < 0) {
    acc = acc + sensorResolution;
    acc = acc + xError;
  } else if (acc > 0 ) {
    acc = acc - sensorResolution;
    acc = acc - xError;
  }
  if (loopIterator == 8) {
    loopIterator = 0;
    light = false;
    Serial.print("Turn "); Serial.println(" light off.");
  }
  val = digitalRead(switch_pin); // check mercury switch state
  if (val == HIGH) {
    if (light) {
      if (sideOn == LEFT && acc < 0) {
        sideOn = RIGHT;
        light = true;
        loopIterator = -1;
        Serial.print("Turn "); Serial.print(sideOn); Serial.println(" light on.");
        turnOffTurnLight(LEFT);
        turnOffTurnLight(RIGHT);
      } else if (sideOn == RIGHT && acc >= 0) {
        sideOn = LEFT;
        light = true;
        loopIterator = -1;
        Serial.print("Turn "); Serial.print(sideOn); Serial.println(" light on.");
        turnOffTurnLight(LEFT);
        turnOffTurnLight(RIGHT);
      }
    } else {
      if (acc < 0) {
        sideOn = RIGHT;
      } else {
        sideOn = LEFT;
      }
      light = true;
      loopIterator = -1;
      Serial.print("Turn "); Serial.print(sideOn); Serial.println(" light on.");
      turnOffTurnLight(LEFT);
      turnOffTurnLight(RIGHT);
    }
  }
  if (light) {
    if (isOn) {
      turnOffTurnLight(sideOn);
      isOn = false;
    } else {
      turnLight(sideOn);
      isOn = true;
    }
  } else {
    turnOffTurnLight(LEFT);
    turnOffTurnLight(RIGHT);
  }
  loopIterator++;
  delay(500);
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
