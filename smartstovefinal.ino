#define BLYNK_PRINT Serial

// --- BLYNK CREDENTIALS ---
#define BLYNK_TEMPLATE_ID " "
#define BLYNK_TEMPLATE_NAME "SmartStove"
#define BLYNK_AUTH_TOKEN " "

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// --- LIBRARIES ---
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <DHT.h>

// --- PIN DEFINITIONS ---
// SENSORS & ACTUATORS (Right Side Pins)
#define SOUND_PIN         5   // Sound Sensor
#define RELAY_PIN         19  // Valve Relay
#define SERVO_PIN         18  // Flame Servo
#define DHT_PIN           17  // Temperature

// ALARMS (Top Left)
#define BUZZER_PIN        33  // SG1 
#define LED_PIN           32  // LED1

// L293D MOTOR DRIVER (Middle Left)
#define MOTOR_EN_PIN      27  // L293D Enable
#define MOTOR_IN1_PIN     26  // L293D Input 1
#define MOTOR_IN2_PIN     25  // L293D Input 2

// BUTTONS (Bottom Left)
#define BTN_POWER_PIN     14  // SW8
#define BTN_WHISTLE_PIN   12  // SW9
#define BTN_PLUS_PIN      13  // SW10

// BUTTONS (Bottom Right)
#define BTN_MINUS_PIN     16  // SW11
#define BTN_SEC_PLUS_PIN  4   // SW12
#define BTN_SEC_MINUS_PIN 23  // SW13
#define BTN_FLAME_PIN     15  // SW14

// THE EXCEPTION
#define MQ2_PIN           34  // Gas Sensor MUST connect to 34

// --- SENSOR SETUP ---
#define DHTTYPE DHT11         
DHT dht(DHT_PIN, DHTTYPE);

// --- OLED SETUP ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- WIFI CREDENTIALS ---
char ssid[] = " ";
char pass[] = " ";

// --- SYSTEM STATE VARIABLES ---
int gasThreshold = 1000;
int gasValue = 0;
bool isStoveOn = false;
bool isGasLeaking = false;
int gasAlarmCount = 0;
const int gasAlarmThreshold = 3;
float tempC = 0.0;

// --- FLAME LEVEL ---
int flameLevel = 2; // 0=LOW, 1=MED, 2=HIGH
const int flameLevelAngles[] = {180, 135, 90};  // LOW=180, MED=135, HIGH=90
const char* flameLevelNames[] = {"LOW", "MED", "HIGH"};

// --- WHISTLE LOGIC ---
int targetWhistles = 0;       
int currentWhistles = 0;

// --- TIMERS & GRACE PERIODS ---
unsigned long previousOledMillis = 0;
const long oledInterval = 1000;
unsigned long previousDhtMillis = 0;
const long dhtInterval = 2000; 

unsigned long lighterStartTime = 0;
bool isLighterActive = false;
bool isWaitingForValve = false;
const long lighterDuration = 2500;

// WHISTLE FILTER
unsigned long lastWhistleCooldownTick = 0;
unsigned long continuousLoudStartTime = 0;
unsigned long lastLoudPulseTick = 0;
bool isSoundActive = false;
const long whistleCooldown = 8000;
const long requiredWhistleDuration = 2000;
const long pulseGapTolerance = 500;

// IGNITION BLINDNESS
unsigned long ignitionGraceStartTime = 0;
const long ignitionGraceDuration = 25000; // Increased to 25s to account for physical push time
bool isIgnitionGracePeriod = false;

// FLAME VERIFICATION
float tempAtIgnition = 0.0;
bool flameCheckPending = false;
unsigned long flameCheckTime = 0;
const long flameCheckDelay = 30000; // Increased to 30s to account for physical push time
const float minTempRise = 2.0;

// --- DEBOUNCING ---
unsigned long lastPowerBtnPress = 0;
unsigned long lastWhistleBtnPress = 0;
unsigned long lastPlusPress = 0;
unsigned long lastMinusPress = 0;
unsigned long lastSecPlusPress = 0;
unsigned long lastSecMinusPress = 0;
unsigned long lastFlamePress = 0;
const long debounceDelay = 250; 

// --- TIMER ---
unsigned long timerRemainingMillis = 0;
unsigned long lastTimerTickMillis = 0;
unsigned long stoveOnStartTime = 0;

enum TimerState { TIMER_OFF, TIMER_RUNNING, TIMER_PAUSED };
TimerState timerState = TIMER_OFF;

unsigned long savedTimerMillis = 0;
bool hasSavedTimer = false;

// --- IGNITION STATE MACHINE ---
enum IgnitionState { IGN_OFF, IGN_PUSHING_KNOB, IGN_TURNING_VALVE, IGN_SPARKING, IGN_RELEASING_KNOB, IGN_COMPLETED };
IgnitionState ignState = IGN_OFF;
unsigned long ignStateStartTime = 0;
unsigned long knobPushDuration = 5000; // <--- 5 SECONDS TO PUSH/RELEASE KNOB (Adjust here!)

// --- SERVO ---
Servo stoveServo;
int servoOffPos = 5;
int currentServoPos = 5;
int targetServoPos = 5;
bool isServoMoving = false;
unsigned long previousServoMillis = 0;
const int servoSpeedDelay = 1;

// Clamp servo to safe max
const int SERVO_MAX = 180;

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- SYSTEM BOOTING ---");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOUND_PIN, INPUT); 
  
  pinMode(BTN_POWER_PIN, INPUT_PULLUP); 
  pinMode(BTN_WHISTLE_PIN, INPUT_PULLUP);
  pinMode(BTN_PLUS_PIN, INPUT_PULLUP);
  pinMode(BTN_MINUS_PIN, INPUT_PULLUP); 
  pinMode(BTN_SEC_PLUS_PIN, INPUT_PULLUP);
  pinMode(BTN_SEC_MINUS_PIN, INPUT_PULLUP);
  pinMode(BTN_FLAME_PIN, INPUT_PULLUP);

  digitalWrite(RELAY_PIN, HIGH); 
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_EN_PIN, LOW);

  dht.begin();
  stoveServo.setPeriodHertz(50); 
  stoveServo.attach(SERVO_PIN, 500, 2400); 
  stoveServo.write(servoOffPos);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("OLED FAILED"));
    for(;;); 
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println("Connecting...");
  display.display();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("--- SYSTEM READY ---");
}

// ==========================================
// CONTROL FUNCTIONS
// ==========================================
void motorForward() {
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_EN_PIN, HIGH);
}

void motorReverse() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
  digitalWrite(MOTOR_EN_PIN, HIGH);
}

void motorStop() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_EN_PIN, LOW);
}

void turnStoveON() {
  if (gasValue >= (gasThreshold / 2)) {
    Serial.println(">>> IGNITION BLOCKED!");
    Blynk.virtualWrite(V0, 0); 
    return; 
  }
  
  isStoveOn = true;
  stoveOnStartTime = millis();
  lastTimerTickMillis = millis();
  
  // BEGIN NEW IGNITION SEQUENCE (Push Knob In First)
  ignState = IGN_PUSHING_KNOB;
  ignStateStartTime = millis();
  motorForward(); 
  Serial.print(">>> IGNITION START: Pushing Knob for "); Serial.print(knobPushDuration / 1000); Serial.println("s");
  
  tempAtIgnition = tempC;
  flameCheckPending = true;
  flameCheckTime = millis() + flameCheckDelay;
  
  isIgnitionGracePeriod = true;
  ignitionGraceStartTime = millis();
  Serial.print(">>> STOVE ON TARGET: Flame="); Serial.println(flameLevelNames[flameLevel]);

  Blynk.virtualWrite(V0, 1);    
  currentWhistles = 0;          
}

void turnStoveOFF() {
  isStoveOn = false;
  
  // Save timer if it was active
  if ((timerState == TIMER_RUNNING || timerState == TIMER_PAUSED) && timerRemainingMillis > 0) {
    savedTimerMillis = timerRemainingMillis;
    hasSavedTimer = true;
    Serial.print(">>> Timer SAVED: "); Serial.print(savedTimerMillis / 1000); Serial.println("s");
  }
  timerState = TIMER_OFF;
  
  targetServoPos = servoOffPos;
  isServoMoving = true;
  isWaitingForValve = false;
  digitalWrite(RELAY_PIN, HIGH);
  
  motorStop(); // Safety shutoff
  ignState = IGN_OFF;
  
  flameCheckPending = false;
  
  currentWhistles = 0;
  Blynk.virtualWrite(V5, currentWhistles);
  Blynk.virtualWrite(V0, 0);     
  Serial.println(">>> STOVE OFF.");
}

// ==========================================
// BLYNK CONTROLS
// ==========================================
BLYNK_CONNECTED() {
  Blynk.virtualWrite(V0, isStoveOn ? 1 : 0);
  Blynk.syncVirtual(V1);   // PULL gas threshold FROM app (keeps last set value)
  Blynk.virtualWrite(V3, currentServoPos);
  Blynk.syncVirtual(V4);   // PULL target whistles FROM app
  Blynk.virtualWrite(V11, flameLevel);
}

BLYNK_WRITE(V0) { if (param.asInt() == 1) turnStoveON(); else turnStoveOFF(); }
BLYNK_WRITE(V1) { gasThreshold = param.asInt(); }
BLYNK_WRITE(V3) { 
  int val = param.asInt();
  if (val > 180) val = 180;
  if (val < 0) val = 0;
  targetServoPos = val; 
  isServoMoving = true; 
}
BLYNK_WRITE(V4) { targetWhistles = param.asInt(); }

// V9: Pause/Play Timer
BLYNK_WRITE(V9) {
  if (param.asInt() == 1) {
    if (timerState == TIMER_RUNNING) {
      timerState = TIMER_PAUSED;
    } else if (timerState == TIMER_PAUSED) {
      timerState = TIMER_RUNNING;
      lastTimerTickMillis = millis();
    } else if (!isStoveOn && hasSavedTimer) {
      turnStoveON();
      // Timer will auto-start after spark
    }
  }
}

// V10: Stop/Clear Timer
BLYNK_WRITE(V10) {
  if (param.asInt() == 1) {
    timerState = TIMER_OFF;
    timerRemainingMillis = 0;
    savedTimerMillis = 0;
    hasSavedTimer = false;
  }
}

// V11: Flame Level
BLYNK_WRITE(V11) {
  int val = param.asInt();
  if (val >= 0 && val <= 2) {
    flameLevel = val;
    if (isStoveOn) {
      targetServoPos = flameLevelAngles[flameLevel];
      isServoMoving = true;
    }
  }
}

// V12: App Timer Minutes Input (use Numeric Input or Slider 0-60)
BLYNK_WRITE(V12) {
  int mins = param.asInt();
  if (mins < 0) mins = 0;
  if (mins > 60) mins = 60;
  // Keep existing seconds, replace minutes portion
  unsigned long currentSecs = (timerRemainingMillis / 1000UL) % 60;
  timerRemainingMillis = (mins * 60000UL) + (currentSecs * 1000UL);
  if (isStoveOn && timerRemainingMillis > 0 && timerState == TIMER_OFF) {
    timerState = TIMER_RUNNING;
    lastTimerTickMillis = millis();
  }
  Serial.print(">>> APP: Timer mins = "); Serial.println(mins);
}

// V13: App Timer Seconds Input (use Numeric Input or Slider 0-59)
BLYNK_WRITE(V13) {
  int secs = param.asInt();
  if (secs < 0) secs = 0;
  if (secs > 59) secs = 59;
  // Keep existing minutes, replace seconds portion
  unsigned long currentMins = timerRemainingMillis / 60000UL;
  timerRemainingMillis = (currentMins * 60000UL) + (secs * 1000UL);
  if (isStoveOn && timerRemainingMillis > 0 && timerState == TIMER_OFF) {
    timerState = TIMER_RUNNING;
    lastTimerTickMillis = millis();
  }
  Serial.print(">>> APP: Timer secs = "); Serial.println(secs);
}

// V14: Motor Manual Control
BLYNK_WRITE(V14) {
  int motorState = param.asInt();
  if (motorState == 1) { motorForward(); Serial.println(">>> MOTOR MANUAL OVERRIDE: PUSH"); } 
  else { motorStop(); Serial.println(">>> MOTOR MANUAL OVERRIDE: STOP"); }
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  Blynk.run();
  unsigned long currentMillis = millis();

  bool isIgniting = (ignState != IGN_OFF && ignState != IGN_COMPLETED);

  // =======================================================
  // 1. PHYSICAL BUTTONS (All blocked during ignition sequence)
  // =======================================================
  if (!isIgniting) {

    // POWER
    if (digitalRead(BTN_POWER_PIN) == LOW && (currentMillis - lastPowerBtnPress > debounceDelay)) {
      lastPowerBtnPress = currentMillis;
      if (isStoveOn) turnStoveOFF(); else turnStoveON();
    }

    // WHISTLE (OFF=set whistles, ON=cycle flame)
    if (digitalRead(BTN_WHISTLE_PIN) == LOW && (currentMillis - lastWhistleBtnPress > debounceDelay)) {
      lastWhistleBtnPress = currentMillis;
      if (isStoveOn) {
        flameLevel++;
        if (flameLevel > 2) flameLevel = 0;
        targetServoPos = flameLevelAngles[flameLevel];
        isServoMoving = true;
        Blynk.virtualWrite(V11, flameLevel);
        Serial.print(">>> Flame: "); Serial.println(flameLevelNames[flameLevel]);
      } else {
        targetWhistles++;
        if (targetWhistles > 10) targetWhistles = 0;
        Blynk.virtualWrite(V4, targetWhistles);
      }
    }

    // +1 MINUTE
    if (digitalRead(BTN_PLUS_PIN) == LOW && (currentMillis - lastPlusPress > debounceDelay)) {
      lastPlusPress = currentMillis;
      timerRemainingMillis += 60000UL;
      if (timerRemainingMillis > 3600000UL) timerRemainingMillis = 3600000UL;
      if (isStoveOn && timerState == TIMER_OFF) { timerState = TIMER_RUNNING; lastTimerTickMillis = currentMillis; }
      Serial.println(">>> +1 min");
    }

    // -1 MINUTE
    if (digitalRead(BTN_MINUS_PIN) == LOW && (currentMillis - lastMinusPress > debounceDelay)) {
      lastMinusPress = currentMillis;
      if (timerRemainingMillis >= 60000UL) {
        timerRemainingMillis -= 60000UL;
      } else {
        timerRemainingMillis = 0;
        timerState = TIMER_OFF;
        savedTimerMillis = 0;
        hasSavedTimer = false;
      }
      Serial.println(">>> -1 min");
    }

    // +10 SECONDS (NEW BUTTON)
    if (digitalRead(BTN_SEC_PLUS_PIN) == LOW && (currentMillis - lastSecPlusPress > debounceDelay)) {
      lastSecPlusPress = currentMillis;
      timerRemainingMillis += 10000UL;
      if (timerRemainingMillis > 3600000UL) timerRemainingMillis = 3600000UL;
      if (isStoveOn && timerState == TIMER_OFF) { timerState = TIMER_RUNNING; lastTimerTickMillis = currentMillis; }
      Serial.println(">>> +10 sec");
    }

    // -10 SECONDS (NEW BUTTON)
    if (digitalRead(BTN_SEC_MINUS_PIN) == LOW && (currentMillis - lastSecMinusPress > debounceDelay)) {
      lastSecMinusPress = currentMillis;
      if (timerRemainingMillis >= 10000UL) {
        timerRemainingMillis -= 10000UL;
      } else {
        timerRemainingMillis = 0;
        timerState = TIMER_OFF;
        savedTimerMillis = 0;
        hasSavedTimer = false;
      }
      Serial.println(">>> -10 sec");
    }

    // FLAME LEVEL BUTTON (NEW - cycles LOW->MED->HIGH->LOW)
    if (digitalRead(BTN_FLAME_PIN) == LOW && (currentMillis - lastFlamePress > debounceDelay)) {
      lastFlamePress = currentMillis;
      flameLevel++;
      if (flameLevel > 2) flameLevel = 0;
      Blynk.virtualWrite(V11, flameLevel);
      Serial.print(">>> Flame: "); Serial.println(flameLevelNames[flameLevel]);
      if (isStoveOn) {
        targetServoPos = flameLevelAngles[flameLevel];
        isServoMoving = true;
      }
    }

    // PAUSE/RESUME: Press BOTH PLUS and MINUS together
    if (digitalRead(BTN_PLUS_PIN) == LOW && digitalRead(BTN_MINUS_PIN) == LOW) {
      static unsigned long lastComboPress = 0;
      if (currentMillis - lastComboPress > 600) {
        lastComboPress = currentMillis;
        if (timerState == TIMER_RUNNING) {
          timerState = TIMER_PAUSED;
          Serial.println(">>> Timer PAUSED");
        } else if (timerState == TIMER_PAUSED) {
          timerState = TIMER_RUNNING;
          lastTimerTickMillis = currentMillis;
          Serial.println(">>> Timer RESUMED");
        } else if (!isStoveOn && hasSavedTimer) {
          turnStoveON();
        }
      }
    }
  }

  // =======================================================
  // 2. WHISTLE SENSOR (Continuous Duration Filter)
  // =======================================================
  if (!isIgniting && digitalRead(SOUND_PIN) == LOW) {
    lastLoudPulseTick = currentMillis; 
    if (!isSoundActive && (currentMillis - lastWhistleCooldownTick > whistleCooldown)) {
      isSoundActive = true;
      continuousLoudStartTime = currentMillis; 
    }
  }

  if (isSoundActive) {
    if (currentMillis - lastLoudPulseTick > pulseGapTolerance) {
      isSoundActive = false;
    } 
    else if (currentMillis - continuousLoudStartTime >= requiredWhistleDuration) {
      isSoundActive = false; 
      lastWhistleCooldownTick = currentMillis;
      Serial.println(">>> TRUE WHISTLE!");
      
      if (isStoveOn) {
        currentWhistles++;
        Blynk.virtualWrite(V5, currentWhistles);
        
        digitalWrite(BUZZER_PIN, HIGH); delay(150); digitalWrite(BUZZER_PIN, LOW); delay(150);
        digitalWrite(BUZZER_PIN, HIGH); delay(150); digitalWrite(BUZZER_PIN, LOW);
        
        if (targetWhistles > 0 && currentWhistles >= targetWhistles) {
          turnStoveOFF();
        }
      }
    }
  }

  // =======================================================
  // 3. READ SENSORS
  // =======================================================
  gasValue = analogRead(MQ2_PIN);
  if (currentMillis - previousDhtMillis >= dhtInterval) {
    previousDhtMillis = currentMillis;
    tempC = dht.readTemperature();
    if (isnan(tempC)) tempC = 0.0; 
  }

  // =======================================================
  // 4. IGNITION SEQUENCE & SERVO
  // =======================================================
  if (ignState == IGN_PUSHING_KNOB && (currentMillis - ignStateStartTime >= knobPushDuration)) {
    motorStop(); // Stop pushing
    ignState = IGN_TURNING_VALVE;
    targetServoPos = flameLevelAngles[flameLevel];
    isServoMoving = true;
    isWaitingForValve = true;
    Serial.println(">>> Knob Pushed in. Turning Valve to release gas...");
  }

  if (isServoMoving && (currentMillis - previousServoMillis >= servoSpeedDelay)) {
    previousServoMillis = currentMillis;
    if (targetServoPos > SERVO_MAX) targetServoPos = SERVO_MAX; // Safety clamp
    if (currentServoPos < targetServoPos) { currentServoPos++; stoveServo.write(currentServoPos); } 
    else if (currentServoPos > targetServoPos) { currentServoPos--; stoveServo.write(currentServoPos); } 
    else { 
      isServoMoving = false; 
      if (ignState == IGN_TURNING_VALVE && isWaitingForValve && currentServoPos == flameLevelAngles[flameLevel]) {
        isWaitingForValve = false;
        ignState = IGN_SPARKING;
        ignStateStartTime = currentMillis;
        digitalWrite(RELAY_PIN, LOW); // Spark ON
        Serial.println(">>> Valve Open. Lighter FIRED!");
      }
    }
  }

  if (ignState == IGN_SPARKING && (currentMillis - ignStateStartTime >= lighterDuration)) {
    digitalWrite(RELAY_PIN, HIGH); // Spark OFF
    ignState = IGN_RELEASING_KNOB;
    ignStateStartTime = currentMillis;
    motorReverse(); // Pull knob back out
    Serial.println(">>> Lighter off. Reversing Motor to Release Knob...");
  }

  if (ignState == IGN_RELEASING_KNOB && (currentMillis - ignStateStartTime >= knobPushDuration)) {
    motorStop();
    ignState = IGN_COMPLETED;
    Serial.println(">>> Knob released. Ignition complete. Timer Starts.");
    
    // START TIMER NOW (after flame is lit and knob released)
    if (hasSavedTimer && savedTimerMillis > 0) {
      timerRemainingMillis = savedTimerMillis;
      timerState = TIMER_PAUSED;
      hasSavedTimer = false;
      savedTimerMillis = 0;
    } else if (timerRemainingMillis > 0) {
      timerState = TIMER_RUNNING;
      lastTimerTickMillis = currentMillis;
    }
  }

  // =======================================================
  // 5. SAFETY SYSTEM
  // =======================================================
  if (isIgnitionGracePeriod) {
    if (currentMillis - ignitionGraceStartTime >= ignitionGraceDuration) {
      isIgnitionGracePeriod = false;
    }
  } else {
    if (gasValue >= (gasThreshold / 2)) {
      digitalWrite(LED_PIN, HIGH);
      if (gasValue >= gasThreshold) {
        gasAlarmCount++;
        if (gasAlarmCount >= gasAlarmThreshold) {
          digitalWrite(BUZZER_PIN, HIGH);
          if (!isGasLeaking) {
            isGasLeaking = true;
            Blynk.logEvent("gas_alert", "CRITICAL: Gas Leak!");
          }
          if (isStoveOn || targetServoPos != servoOffPos) {
            turnStoveOFF();
            targetServoPos = servoOffPos;
          }
        }
      } else {
        gasAlarmCount = 0;
        if (isStoveOn) { turnStoveOFF(); targetServoPos = servoOffPos; }
      }
    } else {
      gasAlarmCount = 0;
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      isGasLeaking = false;
    }
  }

  // FLAME CHECK
  if (flameCheckPending && isStoveOn && currentMillis >= flameCheckTime) {
    flameCheckPending = false;
    if (tempC - tempAtIgnition < minTempRise) {
      Blynk.logEvent("gas_alert", "Flame not detected!");
      for(int b = 0; b < 4; b++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(150);
        digitalWrite(BUZZER_PIN, LOW);  delay(150);
      }
      turnStoveOFF();
    }
  }

  // =======================================================
  // 6. TIMER COUNTDOWN & DEADMAN
  // =======================================================
  if (timerState == TIMER_RUNNING) {
    unsigned long elapsed = currentMillis - lastTimerTickMillis;
    lastTimerTickMillis = currentMillis;

    if (timerRemainingMillis > elapsed) {
      timerRemainingMillis -= elapsed;
    } else {
      timerRemainingMillis = 0;
      timerState = TIMER_OFF;
      savedTimerMillis = 0;
      hasSavedTimer = false;
      for(int b = 0; b < 3; b++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(100); 
        digitalWrite(BUZZER_PIN, LOW);  delay(100);
      }
      turnStoveOFF();
    }
  }

  // Deadman: 2hr auto-shutoff
  if (isStoveOn && timerState == TIMER_OFF) {
    if (currentMillis - stoveOnStartTime >= 7200000UL) { 
      for(int b = 0; b < 5; b++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(200); 
        digitalWrite(BUZZER_PIN, LOW);  delay(200);
      }
      turnStoveOFF();
    }
  }

  // =======================================================
  // 7. OLED & BLYNK DISPLAY
  // =======================================================
  if (currentMillis - previousOledMillis >= oledInterval) {
    previousOledMillis = currentMillis;

    Blynk.virtualWrite(V2, gasValue);               
    Blynk.virtualWrite(V5, currentWhistles);        
    Blynk.virtualWrite(V6, tempC);                  

    // Timer string
    unsigned long displayMillis = timerRemainingMillis;
    if (!isStoveOn && hasSavedTimer) displayMillis = savedTimerMillis;
    
    String timerStr;
    if (displayMillis == 0 && timerState == TIMER_OFF && !hasSavedTimer) {
      timerStr = "MANUAL";
    } else {
      unsigned long totSecs = displayMillis / 1000UL;
      char tBuf[10];
      sprintf(tBuf, "%02lu:%02lu", totSecs / 60, totSecs % 60);
      timerStr = String(tBuf);
      if (timerState == TIMER_PAUSED || (!isStoveOn && hasSavedTimer)) timerStr += " P";
    }

    // Status colors
    String systemStatus = "SYSTEM SAFE";
    String statusColor = "#23C48E";
    if (isGasLeaking) { systemStatus = "DANGER! LEAK!"; statusColor = "#D3435C"; }
    else if (gasValue >= (gasThreshold / 2)) { systemStatus = "WARNING"; statusColor = "#E4C215"; }

    Blynk.virtualWrite(V7, systemStatus);           
    Blynk.setProperty(V7, "color", statusColor);         
    Blynk.virtualWrite(V8, timerStr);

    // --- OLED ---
    display.clearDisplay();
    display.setTextSize(1);
    
    // Row 1: Status | Temp
    display.setCursor(0, 0);
    if (isGasLeaking) display.print("DANGER!");
    else if (gasValue >= (gasThreshold / 2)) display.print("WARNING");
    else display.print("SAFE");
    display.print(" "); display.print(tempC, 1); display.println("C");

    // Row 2: Gas
    display.setCursor(0, 11);
    display.print("Gas:"); display.print(gasValue); display.print("/"); display.print(gasThreshold);

    // Row 3: Stove | Flame | Whistles
    display.setCursor(0, 22);
    display.print(isStoveOn ? "ON" : "OFF");
    display.print(" F:"); display.print(flameLevelNames[flameLevel]);
    display.print(" W:"); display.print(currentWhistles); display.print("/"); display.print(targetWhistles);

    // Row 4: TIMER (large)
    display.setCursor(0, 38);
    display.setTextSize(1);
    
    bool showTimer = (timerState != TIMER_OFF) || 
                     (!isStoveOn && hasSavedTimer && savedTimerMillis > 0) ||
                     timerRemainingMillis > 0;
    
    if (showTimer) {
      unsigned long tMillis = timerRemainingMillis;
      if (!isStoveOn && hasSavedTimer) tMillis = savedTimerMillis;
      
      display.print("T:");
      display.setTextSize(2);
      
      if (timerState == TIMER_PAUSED || (!isStoveOn && hasSavedTimer)) {
        // Blink when paused
        if ((currentMillis / 1000) % 2 == 0) {
          char buf[10];
          unsigned long ts = tMillis / 1000UL;
          sprintf(buf, "%02lu:%02lu", ts / 60, ts % 60);
          display.print(buf);
        }
      } else {
        char buf[10];
        unsigned long ts = tMillis / 1000UL;
        sprintf(buf, "%02lu:%02lu", ts / 60, ts % 60);
        display.print(buf);
      }
    } else {
      display.setTextSize(2);
      display.print("MANUAL");
    }
    
    display.display();
  }
}
