#include "BLEDevice.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>

#define leftStripPin 18
#define rightStripPin 19
#define tilt_sensor_pin 4

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

Adafruit_NeoPixel leftStrip = Adafruit_NeoPixel(8, leftStripPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rightStrip = Adafruit_NeoPixel(8, rightStripPin, NEO_GRB + NEO_KHZ800);
const uint32_t red = rightStrip.Color(255, 0, 0);
const uint32_t orange = rightStrip.Color(255, 180, 0);
const uint32_t offColor = rightStrip.Color(0, 0, 0);
enum Side {
  LEFT, RIGHT
};
const int turnLightsLoop = 5;
class TurnLightData {
  public:
    bool tilt = false;
    Side sideOn = LEFT;
    Adafruit_NeoPixel* strip = NULL;
    short pixels[5] = {0, 0, 0, 0, 0};
    int loopsLeft = turnLightsLoop;

    void reset() {
      strip = NULL;
      resetPixels();
      sideOn = LEFT;
      loopsLeft = turnLightsLoop;
      tilt = false;
    }

    bool isLoopEnd() {
      bool isEnd = true;
      for (int i = 0; i < 5; i++) {
        if (pixels[i] < 7) {
          isEnd = false;
          break;
        }
      }
      return isEnd;
    }

    void resetPixels() {
      for (int i = 0; i < 5; i++) {
        this->pixels[i] = 0;
      }
    }

    void incrementPixels() {
      for (short i = 0; i < 5; i++) {
        if (i > 0 && pixels[i - 1] == 1) {
          pixels[i] = 0;
          continue;
        }
        if (pixels[i] == 7) {
          continue;
        }
        pixels[i] = pixels[i] + 1;
      }
    }
};
TurnLightData turnLightData = TurnLightData();
bool tilt = false;

struct AccelData {
  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
  double xError = 0;
  double sensorResolution = 0;
} accelData;

bool stopLights = false;




class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      Serial.println("onDisconnect");
    }
};

/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());
      Serial.println(advertisedDevice.getServiceUUID().toString().c_str());
      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
        Serial.println("Is our device");
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;
      }
      else if (!advertisedDevice.haveServiceUUID()) {
        Serial.println("Does not have serivce UUID");
      } else {
        Serial.println("Not the service we are looking for");
      }
    }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

void bleLoop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  if (!connected && doScan) {
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
}

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (*pData == 1) {
    stopLights = true;
  } else {
    Serial.print("Unknown data: ");
    Serial.print(length);
    Serial.print(" bytes : ");
    Serial.println(*pData);
  }
}

void tiltDetected() {
  tilt = true;
}

double getXAxisAcceleration() {
  sensors_event_t event;
  accelData.accel.getEvent(&event);
  double acc = event.acceleration.x;
  if (acc < 0) {
    acc = acc + accelData.sensorResolution;
    acc = acc + accelData.xError;
  } else if (acc > 0 ) {
    acc = acc - accelData.sensorResolution;
    acc = acc - accelData.xError;
  }
  return acc;
}

void setPixels(const uint32_t* color, int firstPixel, int lastPixel, Adafruit_NeoPixel* strip) {
  for (short i = firstPixel; i <= lastPixel; i++) {
    strip->setPixelColor(i  , *color);
  }
}

void stopLight() {
  if (turnLightData.tilt) {
    if (turnLightData.sideOn == LEFT) {
      setPixels(&red, 0, 7, &rightStrip);
      rightStrip.show();
    } else {
      setPixels(&red, 0, 7, &leftStrip);
      leftStrip.show();
    }
  } else {
    setPixels(&red, 0, 7, &rightStrip);
    setPixels(&red, 0, 7, &leftStrip);
    rightStrip.show();
    leftStrip.show();
  }
}

void turnOffLights() {
  setPixels(&offColor, 0, 7, &rightStrip);
  setPixels(&offColor, 0, 7, &leftStrip);
  rightStrip.show();
  leftStrip.show();
}

void setup() {
  accelData.accel.begin();
  accelData.accel.setRange(ADXL345_RANGE_2_G);
  sensor_t sensor;
  accelData.accel.getSensor(&sensor);
  accelData.sensorResolution = sensor.resolution;
  pinMode(tilt_sensor_pin, INPUT);
  leftStrip.begin();
  rightStrip.begin();
  attachInterrupt(tilt_sensor_pin, tiltDetected, RISING);

  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void turnLights() {
  if (turnLightData.isLoopEnd()) {
    Serial.println("One loop end");
    turnLightData.loopsLeft--;
    turnLightData.resetPixels();
  }
  for (short i = 0; i < 5; i++) {
    short pixel = turnLightData.pixels[i];
    Serial.print("Setting pixel: "); Serial.print(pixel); Serial.println(" on");
    turnLightData.strip->setPixelColor(pixel, orange);
  }
  for (short i = 0; i < 8; i++) {
    bool in = false;
    for (short j = 0; j < 5; j++) {
      if (turnLightData.pixels[j] == i) {
        in = true;
        break;
      }
    }
    if (!in) {
      Serial.print("Setting pixel: "); Serial.print(i); Serial.println(" off");
      turnLightData.strip->setPixelColor(i, offColor);
    }
  }
  turnLightData.strip->show();
  turnLightData.incrementPixels();
  delay(75);
}

void loop() {
  bleLoop();
  turnOffLights();
  if (stopLights) {
    stopLight();
    stopLights = false;
  }
  if (tilt) {
    turnLightData.tilt = true;
    tilt = false;
  }
  if (turnLightData.tilt) {
    double acc = getXAxisAcceleration();
    Side side = LEFT;
    if (acc < 0) {
      side = RIGHT;
    } else {
      side = LEFT;
    }
    if (turnLightData.sideOn != side) {
      turnLightData.reset();
      turnLightData.sideOn = side;
    }
    if (turnLightData.sideOn == LEFT) {
      turnLightData.strip = &leftStrip;
    } else {
      turnLightData.strip = &rightStrip;
    }
    if (turnLightData.loopsLeft == 0) {
      turnLightData.reset();
    } else {
      turnLights();
    }
  }
}
