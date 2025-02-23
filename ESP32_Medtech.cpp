#include <BluetoothSerial.h>
#include <ESP32Servo.h>
 
BluetoothSerial SerialBT;
 
Servo myServo;
Servo myServo2;
const int servo1Pin = 14;
const int servo2Pin = 27;
 
void setup() {
  // Start the Serial Monitor and Bluetooth Serial
  Serial.begin(115200);
  SerialBT.begin("ESP32_MedTech"); // Name of the ESP32 Bluetooth device
  // Initialize servos
  myServo.setPeriodHertz(50);  // 50Hz for servo movement
  myServo2.setPeriodHertz(50);
  myServo.attach(servo1Pin, 500, 2500);
  myServo2.attach(servo2Pin, 500, 2500);
 
  Serial.println("Bluetooth Serial is ready, connected devices can send data!");
}
 
void loop() {
  // Check if there is data available to read from Bluetooth
  if (SerialBT.available()) {
    String received = SerialBT.readString();
    int angle = received.toInt();
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      myServo2.write(180 - angle);  // Inverse angle for servo2
      Serial.print("Set servos to: ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid angle, please send an integer between 0 and 180.");
    }
  }
 
  // Send a heartbeat message to the connected Bluetooth device
  SerialBT.println("Heartbeat message to ensure communication.");
  delay(1000);  // Repeat every 1 second
}