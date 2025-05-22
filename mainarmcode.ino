#include <Servo.h>
#include <AES.h>
#include <Adafruit_NeoPixel.h>

#define dirPin 10
#define stepPin 9


const int maxSteps = 150;
const int microstepsPerStep = 16;
const int maxMicrosteps = maxSteps * microstepsPerStep; 
int currentStep = 0;  // Tracks current position (0–150)


Servo servo1;
Servo servo2;
Servo servo3;

#define pinServo1 8
#define pinServo2 7
#define pinServo3 6

// Record section variables
bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

const int moveCount = 30;
const int servoNumber = 4;
int movesServos[moveCount][servoNumber]; // max of 10 moves
int servos[servoNumber] = {1, 2, 3, 4};

#define NUM_LEDS 16
#define LED_PIN 13
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
bool ledState = false;


AES128 aes;
byte key[16];

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  servo3.attach(pinServo3);


  strip.begin();
  strip.show(); 

  servo1.write(90);
  servo2.write(90);
  clawOpen();

  Serial.begin(9600);
  
  // Setup AES key: same as Unity "1234567890abcdef"
  const char* keyString = "1234567890abcdef";
  memcpy(key, keyString, 16);
  aes.setKey(key, sizeof(key));
}

void loop() {

  if(!ledState){
    strip.fill(strip.Color(0, 0, 255)); // Full blue
    strip.show();

    ledState = true;
  } 

  processSerialCommand();
    
  if (isPlay) {
    playRecord();
  }
  
  delay(5);
}

void processSerialCommand() {
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check for encrypted messages first
    if (input.length() >= 32 && input.indexOf(":") == -1) {
      // Likely an encrypted message
      String decryptedMessage = decryptHexString(input);
      Serial.print("Received: ");
      Serial.print(input);
      Serial.print("  Decrypted message: ");
      Serial.println(decryptedMessage);
      input = decryptedMessage;
    }
    
    input.trim();
    
    if (input.startsWith("ENC")) {
      handleEncoderCommand(input);
    }
    else if (input == "openButton") {
      clawOpen();
    }
    else if (input == "closeButton") {
      clawClose();
    }
    else if (input == "startRecord") {
      startRecording();
    }
    else if (input == "stopRecord") {
      stopRecording();
    }
    else if (input == "playRecord") {
      startPlayback();
    }
    else if (input == "moveToStart") {
      resetPosition();
    }
    else {
      // Handle Bluetooth-style commands
      String leftSide = splitStringLR(input,0);
      String rightSide = splitStringLR(input,1);
      
      if(leftSide.equals("STEPPER")){
        int angle = rightSide.toInt();
        moveBaseStepper(angle);
      }
      else if(leftSide.equals("SERVO1")){
        int angle = rightSide.toInt();
        servo1.write(angle);
      }
      else if(leftSide.equals("SERVO2")){
        int angle = rightSide.toInt();
        servo2.write(angle);
      }
      else if(leftSide.equals("SERVO3")){
        int angle = rightSide.toInt();
        if(angle<105) angle = 105;
        servo3.write(angle);
      }
      else if(leftSide.equals("SERVOS")){
        moveServosBySave(rightSide);
      }
    }
  }
}

void handleEncoderCommand(String input) {
  int colonIndex = input.indexOf(':');
  if (colonIndex != -1) {
    String servoIdentifier = input.substring(0, colonIndex);
    int angle = input.substring(colonIndex + 1).toInt();
    angle = constrain(angle, 0, 180); // Safety constraint

    if (servoIdentifier == "ENC1") {
      moveBaseStepper(angle);
      if (isRecord) recordMove();
    }
    else if (servoIdentifier == "ENC2") {
      servo1.write(angle);
      if (isRecord) recordMove();
    }
    else if (servoIdentifier == "ENC3") {
      servo2.write(angle);
      if (isRecord) recordMove();
    }
  }
}

// Recording control functions
void startRecording() {
  isRecord = true;
  isPlay = false;
  indexRecord = 0;
  memset(movesServos, 0, sizeof(movesServos));

  Serial.println("Recording started");
}

void stopRecording() {
  isRecord = false;
  Serial.println("Recording stopped");
}

void startPlayback() {
  isPlay = true;
  isRecord = false;
  Serial.println("Playback started");
}

void recordMove() {
  if (isRecord && indexRecord < moveCount) {
    movesServos[indexRecord][0] = currentStep;    // Stepper position
    movesServos[indexRecord][1] = servo1.read();  // Servo1 position
    movesServos[indexRecord][2] = servo2.read();  // Servo2 position
    movesServos[indexRecord][3] = servo3.read();  // Servo3 position
    indexRecord++;
    Serial.print("Move recorded: ");
    Serial.println(indexRecord);
  }
}

void playRecord() {
  if (indexRecord == 0) return;
  
  Serial.println("Starting playback...");
  
  for (int i = 0; i < indexRecord; i++) {
    Serial.print("Playing move ");
    Serial.println(i+1);
    
    // Move stepper
    moveBaseStepper(movesServos[i][0]);
    
    // Move servos
    servo1.write(movesServos[i][1]);
    servo2.write(movesServos[i][2]);
    servo3.write(movesServos[i][3]);
    
    // Wait before next move (adjust as needed)
    delay(500);
  }
  
  isPlay = false;
  Serial.println("Playback completed");
}

void goToStartPosition() {
  if (indexRecord > 0) {
    moveBaseStepper(movesServos[0][0]);
    servo1.write(movesServos[0][1]);
    servo2.write(movesServos[0][2]);
    if (movesServos[0][3] == 1) clawOpen();
    else clawClose();
  }
}

// Claw control functions
void clawOpen() {
  servo3.write(105);
  Serial.println("Claw opened");
  if (isRecord) recordMove();  // Save claw open state during recording
}

void clawClose() {
  servo3.write(177);
  Serial.println("Claw closed");
  if (isRecord) recordMove();  // Save claw close state during recording
}

void moveBaseStepper(int input){
  int targetStep = input;

  if (targetStep < 0 || targetStep > maxSteps) {
    Serial.println("Invalid input! Please enter a value between 0 and 150.");
    return;
  }

  // Map 0-150 input to microsteps
  int targetMicrosteps = targetStep * microstepsPerStep;
  int currentMicrosteps = currentStep * microstepsPerStep;
  int stepsToMove = abs(targetMicrosteps - currentMicrosteps);

  if (stepsToMove == 0) {
    Serial.println("Already at target position.");
    return;
  }

  digitalWrite(dirPin, targetMicrosteps > currentMicrosteps ? HIGH : LOW);

  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);  // Adjust this to control speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
    delay(5);
  }

  currentStep = targetStep;
  Serial.print("Moved to step ");
  Serial.println(currentStep);
}

String decryptHexString(String encHex) {
  // Trim any extra whitespace or \r from the input
  encHex.trim();

  // Convert the hex string to a byte array
  int len = encHex.length() / 2;
  byte cipher[len];
  for (int i = 0; i < len; i++) {
    String byteStr = encHex.substring(i * 2, i * 2 + 2);
    cipher[i] = strtoul(byteStr.c_str(), nullptr, 16);
  }

  // Prepare a byte array for the decrypted data
  byte decrypted[len];
  for (int i = 0; i < len; i += 16) {
    aes.decryptBlock(decrypted + i, cipher + i);
  }

  // Remove PKCS7 padding
  int pad = decrypted[len - 1];
  int realLen = len - pad;

  // Build and return the decrypted string
  String decryptedString = "";
  for (int i = 0; i < realLen; i++) {
    decryptedString += (char)decrypted[i];
  }

  return decryptedString;
}

String splitStringLR(String input, int index) {
  char delimiter = ':';  // Fixed delimiter
  int start = 0, end = 0, count = 0;
  
  while (count <= index) {
    end = input.indexOf(delimiter, start);  // Find next delimiter position
    if (end == -1) end = input.length();    // If no more delimiters, take remaining string
    
    if (count == index) {
      return input.substring(start, end);  // Extract desired substring
    }
    
    start = end + 1;  // Move start position after delimiter
    count++;
  }
  
  return "";  // Return empty string if index is out of range
}

void moveServosBySave(String input){
  int angle1 = splitRightSide(input,0).toInt();
  moveBaseStepper(angle1);
  delay(500);

  int angle2 = splitRightSide(input,1).toInt();
  servo1.write(angle2);
  delay(500);

  int angle3 = splitRightSide(input,2).toInt();
  servo2.write(angle3);
  delay(500);

  int angle4 = splitRightSide(input,3).toInt();
  if(angle4<105) {
    angle4 = 105;
  }
  servo3.write(angle4);
}

String splitRightSide(String input, int index){
  char delimiter = ',';  // Fixed delimiter
  int start = 0, end = 0, count = 0;
  
  while (count <= index) {
    end = input.indexOf(delimiter, start);  // Find next delimiter position
    if (end == -1) end = input.length();    // If no more delimiters, take remaining string
    
    if (count == index) {
      return input.substring(start, end);  // Extract desired substring
    }
    
    start = end + 1;  // Move start position after delimiter
    count++;
  }
  
  return "";  // Return empty string if index is out of range
}

void resetPosition() {
  moveBaseStepper(0);
  servo1.write(90);
  servo2.write(90);
  servo3.write(105);
}

