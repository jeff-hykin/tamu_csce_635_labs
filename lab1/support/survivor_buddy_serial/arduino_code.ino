#include <Servo.h>

#define BAUD_RATE 115200

int baseServoPin = 2;
int baseServo2Pin = 3;
int torsoServoPin = 4;
int headRotServoPin = 5;
int headTiltServoPin = 6;
Servo baseServo1;
Servo baseServo2;
Servo torsoServo;
Servo headRotServo;
Servo headTiltServo;

int resetPos = 90;
int basePos = resetPos;
int torsoPos = resetPos;
int headRotPos = resetPos;
int headTiltPos = resetPos;
int incrementAmount = 1;
String in = "";
const int ledPin = 13;

void setup() {
  Serial.begin(BAUD_RATE);
  baseServo1.attach(baseServoPin);
  baseServo2.attach(baseServo2Pin);
  torsoServo.attach(torsoServoPin);
  headRotServo.attach(headRotServoPin);
  headTiltServo.attach(headTiltServoPin);
  pinMode(ledPin, OUTPUT);
  
  // Serial.println("Send 1 to start");
  // while (Serial.read() != '1'){
  //   digitalWrite(ledPin, HIGH);
  //   delay(200);
  //   digitalWrite(ledPin, LOW);
  //   delay(200);
  // }
  Serial.println("Started");
  writeAllServos();
}

void loop() {
  if (Serial.available())
    in = Serial.readStringUntil('\n');
  else
    in = "";

  
  // 123124125126
  if (in.equals("") == false){
//    Serial.println(in);
    if (in.length() == 12){
      basePos = in.substring(0,3).toInt();
      torsoPos = in.substring(3,6).toInt();
      headRotPos = in.substring(6,9).toInt();
      headTiltPos = in.substring(9,12).toInt();
      printAllServoPos();
    }
    else {
      Serial.println("printing");
      Serial.println(in);
    }
  }
  writeAllServos();
}

void writeAllServos(){
  baseServo1.write(basePos);
  baseServo2.write(180-basePos);
  torsoServo.write(torsoPos);
  headRotServo.write(headRotPos);
  headTiltServo.write(headTiltPos);
}

void printAllServoPos(){
  Serial.print("Base: ");
  Serial.print(basePos);
  Serial.print(", Torso: ");
  Serial.print(torsoPos);
  Serial.print(", Rot: ");
  Serial.print(headRotPos);
  Serial.print(", Tilt: ");
  Serial.println(headTiltPos);
}
