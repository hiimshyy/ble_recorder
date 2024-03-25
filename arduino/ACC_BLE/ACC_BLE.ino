#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

float aX, aY, aZ, gX, gY, gZ; // max. 5 characters String each (if negative)
unsigned long myTime; // max. 10 characters String
const String del = ";"; // 1 character
const int strLen = 10 + 6 * 5 + 6; // + 3 for 3 delimiters

// check https://www.uuidgenerator.net/ to generate your own unique UUIDs
BLEService accelService("30a677f1-f653-4600-92dd-4ea3d13432f0");
BLEStringCharacteristic accelCharacteristic("30a677f1-f653-4600-92dd-4ea3d13432f0", BLERead | BLENotify, strLen);

void setup() {
  IMU.begin();
  Serial.begin(9600);
  // while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

    if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }

  BLE.setLocalName("Arduino Accelerometer");
  BLE.setAdvertisedService(accelService);
  accelService.addCharacteristic(accelCharacteristic);
  BLE.addService(accelService);
  accelCharacteristic.writeValue("1111111111;-0.00;-0.00;-0.00");
  BLE.advertise();

  Serial.println("Bluetooth device is now active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      // delay(1);

      read_IMU();
      myTime = millis();

      String msg = get_message();
      accelCharacteristic.writeValue(msg);

      print_to_serial();
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

void read_IMU() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(aX, aY, aZ);
    IMU.readGyroscope(gX, gY, gZ);
  }
  aX = (aX + 4.0) / 8.0;
  aY = (aY + 4.0) / 8.0;
  aZ = (aZ + 4.0) / 8.0;
  gX = (gX + 2000.0) / 4000.0;
  gY = (gY + 2000.0) / 4000.0;
  gZ = (gZ + 2000.0) / 4000.0;
}

void print_to_serial() {
  Serial.print(myTime); Serial.print('\t');
  Serial.print(aX); Serial.print('\t');
  Serial.print(aY); Serial.print('\t');
  Serial.print(aZ); Serial.print('\n');
  Serial.print(gX); Serial.print('\t');
  Serial.print(gY); Serial.print('\t');
  Serial.print(gZ); Serial.print('\n');
}

String get_message() {
  return String(myTime) + del + String(aX) + del + String(aY) + del + String(aZ) + del + String(gX) + del + String(gY) + del + String(gZ); 
}