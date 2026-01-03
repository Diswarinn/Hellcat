// ---------------------------------------------------------------- //
// ESP32 BLE HYBRID CAR V6.4 (Smart Reverse + Object Avoidance)
// ---------------------------------------------------------------- //

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h> 

// --- UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// --- Pin Definitions ---

// Right Side Motors 
const int ENA = 13; 
const int IN1 = 14; 
const int IN2 = 12; 

// Left Side Motors 
const int ENB = 25; 
const int IN3 = 26; 
const int IN4 = 27; 

// PWM Properties
const int FREQ = 30000;
const int RES = 8;
const int CH_A = 0; 
const int CH_B = 1; 

// --- SENSOR PINS ---
const int TRIG_F = 5;   const int ECHO_F = 18;
const int TRIG_FL = 19; const int ECHO_FL = 21;
const int TRIG_FR = 4;  const int ECHO_FR = 16; 
const int TRIG_L = 32;  const int ECHO_L = 35; 
const int TRIG_R = 22;  const int ECHO_R = 23; 

// --- State & Tuning ---
volatile bool isAutonomous = false; 
int manualSpeed = 150; 
const int STOP_DIST = 25;
const int TURN_DIST = 35;
const int SIDE_DIST = 15;
int autoSpeed = 140; 

// --- Forward Declarations ---
void runAutonomousLogic();
void smartDelay(int ms);

// --- Classes ---
class Motor {
  private: int enPin, in1Pin, in2Pin, pwmCh;
  public:
    Motor(int en, int in1, int in2, int ch) { enPin=en; in1Pin=in1; in2Pin=in2; pwmCh=ch; }
    void init() {
      pinMode(in1Pin, OUTPUT); pinMode(in2Pin, OUTPUT);
      #if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(enPin, FREQ, RES);
      #else
        ledcAttachPin(enPin, pwmCh); ledcSetup(pwmCh, FREQ, RES);
      #endif
      drive(0);
    }
    void drive(int speed, bool reverse = false) {
      if (speed > 255) speed = 255; if (speed < 0) speed = 0;
      if (speed == 0) {
        digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW);
        #if ESP_ARDUINO_VERSION_MAJOR >= 3
          ledcWrite(enPin, 0);
        #else
          ledcWrite(pwmCh, 0);
        #endif
        return;
      }
      if (!reverse) { digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW); }
      else { digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH); }
      #if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(enPin, speed);
      #else
        ledcWrite(pwmCh, speed);
      #endif
    }
};

class Sonar {
  private: int trig, echo;
  public:
    Sonar(int t, int e) { trig = t; echo = e; }
    void init() { pinMode(trig, OUTPUT); pinMode(echo, INPUT); }
    int getDistance() {
      digitalWrite(trig, LOW); delayMicroseconds(2);
      digitalWrite(trig, HIGH); delayMicroseconds(10);
      digitalWrite(trig, LOW);
      long duration = pulseIn(echo, HIGH, 15000);
      if (duration == 0) return 200;
      int cm = duration * 0.034 / 2;
      return (cm > 200) ? 200 : cm;
    }
};

Motor motorR(ENA, IN1, IN2, CH_A);
Motor motorL(ENB, IN3, IN4, CH_B);
Sonar sonarF(TRIG_F, ECHO_F);
Sonar sonarFL(TRIG_FL, ECHO_FL);
Sonar sonarFR(TRIG_FR, ECHO_FR);
Sonar sonarL(TRIG_L, ECHO_L);
Sonar sonarR(TRIG_R, ECHO_R);

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false; 
      motorR.drive(0); motorL.drive(0); 
      pServer->getAdvertising()->start(); 
    };
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str(); 
      if (value.length() > 0) {
        char cmd = value[0];
        
        if (cmd == 'X' || cmd == 'x') { isAutonomous = true; return; }
        if (cmd == 'S' || cmd == 's') { isAutonomous = false; motorR.drive(0); motorL.drive(0); return; }

        int newSpeed = -1;
        if (cmd == '0') newSpeed = 0;
        else if (cmd == '1') newSpeed = 100;
        else if (cmd == '2') newSpeed = 120;
        else if (cmd == '3') newSpeed = 140;
        else if (cmd == '4') newSpeed = 160;
        else if (cmd == '5') newSpeed = 180;
        else if (cmd == '6') newSpeed = 200;
        else if (cmd == '7') newSpeed = 220;
        else if (cmd == '8') newSpeed = 240;
        else if (cmd == '9') newSpeed = 250;
        else if (cmd == 'q') newSpeed = 255;

        if (newSpeed != -1) {
            manualSpeed = newSpeed;
            autoSpeed = newSpeed; 
        }
  
        if (!isAutonomous) {
          if (cmd == 'F') { motorR.drive(manualSpeed); motorL.drive(manualSpeed); }
          else if (cmd == 'B') { motorR.drive(manualSpeed, true); motorL.drive(manualSpeed, true); }
          else if (cmd == 'L') { motorR.drive(manualSpeed, true); motorL.drive(manualSpeed); } 
          else if (cmd == 'R') { motorR.drive(manualSpeed); motorL.drive(manualSpeed, true); } 
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  motorR.init(); motorL.init();
  sonarF.init(); sonarFL.init(); sonarFR.init(); sonarL.init(); sonarR.init();

  BLEDevice::init("ESP32_HELLCAT_DISWARIN");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("System Ready. Waiting for App...");
}

void loop() {
  if (isAutonomous) runAutonomousLogic();
}

void runAutonomousLogic() {
  int dF = sonarF.getDistance();
  int dFL = sonarFL.getDistance();
  int dFR = sonarFR.getDistance();
  int dL = sonarL.getDistance();
  int dR = sonarR.getDistance();

  // --- DEBUG OUTPUT ---
  Serial.print("F:"); Serial.print(dF);
  Serial.print(" L:"); Serial.print(dL);
  Serial.print(" R:"); Serial.print(dR);
  Serial.println();

  // ==========================================
  //      SMART REVERSE & OBSTACLE AVOIDANCE
  // ==========================================
  
  if (dF < STOP_DIST || dFL < SIDE_DIST || dFR < SIDE_DIST) {
      // 1. FULL STOP
      motorR.drive(0); motorL.drive(0); 
      smartDelay(200); 
      if(!isAutonomous) return;

      // 2. DECIDE HOW TO REVERSE
      bool forceTurnLeft = false;
      bool forceTurnRight = false;

      // CASE A: WIDE OPEN (Both > 70)
      if (dL > 70 && dR > 70) {
           Serial.println("Path Clear: Reverse Straight");
           motorR.drive(autoSpeed, true); 
           motorL.drive(autoSpeed, true);
           smartDelay(400); // Standard backup
      }
      // CASE B: LEFT IS CLEARER (Back up into Left side)
      else if (dL > dR) {
           Serial.println("Left Clear: Reverse Tail Left");
           // To move Tail Left: Right Motor Rev FAST, Left Motor Rev SLOW
           motorR.drive(autoSpeed, true); 
           motorL.drive(autoSpeed / 3, true); 
           smartDelay(500);
           forceTurnLeft = true; // We are now angled, we need to spin Left to face front
      }
      // CASE C: RIGHT IS CLEARER (Back up into Right side)
      else {
           Serial.println("Right Clear: Reverse Tail Right");
           // To move Tail Right: Left Motor Rev FAST, Right Motor Rev SLOW
           motorR.drive(autoSpeed / 3, true); 
           motorL.drive(autoSpeed, true);
           smartDelay(500);
           forceTurnRight = true; // We are now angled, we need to spin Right to face front
      }

      // 3. STOP AFTER REVERSE
      if(!isAutonomous) return;
      motorR.drive(0); motorL.drive(0);
      smartDelay(200);

      // 4. PERFORM THE TURN
      // If we did a Smart Reverse, we use the Force flags.
      // If we did a Straight Reverse, we calculate based on sensors.
      
      if (forceTurnLeft) {
          motorR.drive(200, true); motorL.drive(200); // Spin Left
      } 
      else if (forceTurnRight) {
          motorR.drive(200); motorL.drive(200, true); // Spin Right
      }
      else {
          // Standard Turn Logic (for straight backup)
          if ((dFL + dL) > (dFR + dR)) { 
             motorR.drive(200, true); motorL.drive(200); // Left
          } else { 
             motorR.drive(200); motorL.drive(200, true); // Right
          }
      }
      smartDelay(500); 
      motorR.drive(0); motorL.drive(0);
  }
  
  // ==========================================
  //          NORMAL NAVIGATION
  // ==========================================
  else if (dFL < TURN_DIST) { motorR.drive(220); motorL.drive(100); } 
  else if (dFR < TURN_DIST) { motorR.drive(100); motorL.drive(220); } 
  else if (dL < SIDE_DIST) { motorR.drive(220); motorL.drive(autoSpeed); } 
  else if (dR < SIDE_DIST) { motorR.drive(autoSpeed); motorL.drive(220); } 
  else { motorR.drive(autoSpeed); motorL.drive(autoSpeed); }
  
  smartDelay(50);
}

void smartDelay(int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if(!isAutonomous) break; 
    delay(10); 
  }
}