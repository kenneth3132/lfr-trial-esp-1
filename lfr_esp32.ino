#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define SDA_PIN 21
#define SCL_PIN 22
#define MUX_SIG  34   
#define MUX_S0   25
#define MUX_S1   26
#define MUX_S2   27
#define MUX_S3   14
const int SENSOR_COUNT = 13;
#define IN1 4
#define IN2 16
#define IN3 5
#define IN4 2
#define ENA 17   
#define ENB 15    
#define BTN_UP     32
#define BTN_DOWN   33
#define BTN_LEFT   18
#define BTN_RIGHT  19
#define BTN_MIDDLE 23
float Kp = 2;
float Ki = 0;
float Kd = 2;
unsigned long lostStartTime = 0;
const unsigned long lostTimeout = 30;
int sensorValues[13];
int weights[13] = {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300}; 
int position;
int lastError = 0;
float integral = 0;
int baseSpeed = 120;
int fullSpeed = 255;
int errorWindow = 50;
bool isRunning = false;
#define MENU_MAIN 0
#define MENU_SETTINGS 1
int currentMenu = MENU_MAIN;
int mainMenuSelection = 0;  
int settingsMenuSelection = 0;  
int trackMode = 0;          
int speedValue = 150;      
int sensorThreshold = 600;  
int KpValue = 5;           
int KdValue = 15;           
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

int pwmA = 0;
int pwmB = 1;
const int pwmFreq = 10000;
const int pwmRes = 8;

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  display.clearDisplay();
  display.display();
  // Initialize EEPROM
  EEPROM.begin(128);
  loadSettings();
  // MUX select pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  // PWM setup
  ledcSetup(pwmA, pwmFreq, pwmRes);
  ledcSetup(pwmB, pwmFreq, pwmRes);
  ledcAttachPin(ENA, pwmA);
  ledcAttachPin(ENB, pwmB);
  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_MIDDLE, INPUT_PULLUP);

  stopMotors();
  displayMainMenu();
}

void loadSettings() {
  trackMode = EEPROM.read(0);
  if (trackMode > 1) trackMode = 0;
  speedValue = EEPROM.read(1);
  if (speedValue < 50 || speedValue > 255) speedValue = 120;
  EEPROM.get(2, sensorThreshold);
  if (sensorThreshold < 50 || sensorThreshold > 2000) sensorThreshold = 200;
  EEPROM.get(4, KpValue);
  if (KpValue < 0 || KpValue > 100) KpValue = 2;
  EEPROM.get(6, KdValue);
  if (KdValue < 0 || KdValue > 100) KdValue = 2;
  baseSpeed = speedValue;
  fullSpeed = constrain(speedValue + 50, 50, 255);
  Kp = KpValue;  
  Kd = KdValue; 
  
}

void saveSettings() {
  EEPROM.write(0, trackMode);
  EEPROM.write(1, speedValue);
  EEPROM.put(2, sensorThreshold);
  EEPROM.put(4, KpValue);
  EEPROM.put(6, KdValue);
  EEPROM.commit();
  
  baseSpeed = speedValue;
  fullSpeed = constrain(speedValue + 10, 50, 255);
  Kp = KpValue;  
  Kd = KdValue;  
}

void loop() {
  if (isRunning) {
    runLineFollower();
  } else {
    handleMenu();
  }
}

void runLineFollower() {
  readSensors();
  bool allBlack = true;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] == 0) { allBlack = false; break; }
  }
  if (allBlack) {
    stopMotors();
    isRunning = false;
    displayMainMenu();
    return;
  }
  bool allOff = true;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] == 1) { allOff = false; break; }
  }

  if (allOff) {
    if (lostStartTime == 0) lostStartTime = millis();

    unsigned long gap = millis() - lostStartTime;

    if (gap < lostTimeout) {
      setMotorSpeed(fullSpeed, fullSpeed);
    } else {
      if (lastError > 0) {
        spinRightslow();
      } else {
        spinLeftslow();
      }
      return;
    }
  } else {
    lostStartTime = 0;
  }

  position = calculatePosition();

      int error = position - 700;

    if (abs(error) <= errorWindow) {
      setMotorSpeed(fullSpeed, fullSpeed);
    } else {
      integral += error;
      int derivative = error - lastError;
      int corr = Kp * error + Ki * integral + Kd * derivative;
      int L = constrain(baseSpeed + corr, 0, 255);
      int R = constrain(baseSpeed - corr, 0, 255);

      setMotorSpeed(L, R);
    }
    lastError = error;
  
  if (digitalRead(BTN_MIDDLE) == LOW) {
    delay(50);
    if (digitalRead(BTN_MIDDLE) == LOW) {
      stopMotors();
      isRunning = false;
      displayMainMenu();
      delay(300);
    }
  }

}

void handleMenu() {
  if (millis() - lastDebounceTime > debounceDelay) {
    
    // UP button
    if (digitalRead(BTN_UP) == LOW) {
      if (currentMenu == MENU_MAIN) {
        mainMenuSelection = (mainMenuSelection == 0) ? 1 : 0;
        displayMainMenu();
      } else if (currentMenu == MENU_SETTINGS) {
        settingsMenuSelection = (settingsMenuSelection - 1 + 6) % 6;  
        displaySettingsMenu();
      }
      lastDebounceTime = millis();
    }

    else if (digitalRead(BTN_DOWN) == LOW) {
      if (currentMenu == MENU_MAIN) {
        mainMenuSelection = (mainMenuSelection + 1) % 2;
        displayMainMenu();
      } else if (currentMenu == MENU_SETTINGS) {
        settingsMenuSelection = (settingsMenuSelection + 1) % 6; 
        displaySettingsMenu();
      }
      lastDebounceTime = millis();
    }
     
    else if (digitalRead(BTN_LEFT) == LOW && currentMenu == MENU_SETTINGS) {
      switch (settingsMenuSelection) {
        case 0: 
          trackMode = (trackMode == 0) ? 1 : 0;
          break;
        case 1: 
          speedValue = constrain(speedValue - 10, 50, 255);
          break;
        case 2: 
          sensorThreshold = constrain(sensorThreshold - 50, 50, 2000);
          break;
        case 3: 
          KpValue = constrain(KpValue - 1, 0, 50);
          break;
        case 4:
          KdValue = constrain(KdValue - 1, 0, 50);
          break;
      }
      displaySettingsMenu();
      lastDebounceTime = millis();
    }
    
    else if (digitalRead(BTN_RIGHT) == LOW && currentMenu == MENU_SETTINGS) {
      switch (settingsMenuSelection) {
        case 0: 
          trackMode = (trackMode == 0) ? 1 : 0;
          break;
        case 1: 
          speedValue = constrain(speedValue + 10, 50, 255);
          break;
        case 2:
          sensorThreshold = constrain(sensorThreshold + 50, 50, 2000);
          break;
        case 3: 
          KpValue = constrain(KpValue + 1, 0, 50);
          break;
        case 4: 
          KdValue = constrain(KdValue + 1, 0, 50);
          break;
      }
      displaySettingsMenu();
      lastDebounceTime = millis();
    }
    
    else if (digitalRead(BTN_MIDDLE) == LOW) {
      if (currentMenu == MENU_MAIN) {
        if (mainMenuSelection == 0) {
          isRunning = true;
          lastError = 0;
          integral = 0;
          displayRunningScreen();
          delay(500);
        } else if (mainMenuSelection == 1) {
          currentMenu = MENU_SETTINGS;
          settingsMenuSelection = 0;
          displaySettingsMenu();
        }
      } else if (currentMenu == MENU_SETTINGS) {
        if (settingsMenuSelection == 5) {   
          saveSettings();
          currentMenu = MENU_MAIN;
          displayMainMenu();
        }
      }
      lastDebounceTime = millis();
    }
  }
}

void displayMainMenu() {
  display.clearDisplay();
  
  // Title
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 5);
  display.println("Team Gearheads");
  
  // Options
  display.setTextSize(1);
  
  // Start option
  display.setCursor(20, 30);
  if (mainMenuSelection == 0) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.println("START");
  
  // Settings option
  display.setCursor(20, 45);
  if (mainMenuSelection == 1) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.println("SETTINGS");
  
  display.display();
}

void displaySettingsMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Track Mode
  display.setCursor(5, 5);
  if (settingsMenuSelection == 0) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.print("Track: ");
  display.println(trackMode == 0 ? "BLACK" : "WHITE");
  
  // Speed
  display.setCursor(5, 15);
  if (settingsMenuSelection == 1) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.print("Speed: ");
  display.println(speedValue);
  
  // Threshold
  display.setCursor(5, 25);
  if (settingsMenuSelection == 2) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.print("Threshold: ");
  display.println(sensorThreshold);
  
  // Kp
  display.setCursor(5, 35);
  if (settingsMenuSelection == 3) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.print("Kp: ");
  display.println(KpValue);
  
  // Kd
  display.setCursor(5, 45);
  if (settingsMenuSelection == 4) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.print("Kd: ");
  display.println(KdValue);
  
  // OK button
  display.setCursor(5, 55);
  if (settingsMenuSelection == 5) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.println("OK"); 
  
  display.display();
}

void displayRunningScreen() {
  display.clearDisplay();
  
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 20);
  display.println("RUNNING");
  
  display.setTextSize(1);
  display.setCursor(10, 45);
  display.print("Kp:");
  display.print(Kp);
  display.print(" Kd:");
  display.print(Kd);
  
  display.display();
}
int readMuxChannel(int channel) {
  digitalWrite(MUX_S0, channel & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);
  delayMicroseconds(50);
  return analogRead(MUX_SIG);
}

void readSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int raw = readMuxChannel(i);
    if (trackMode == 0) {
      sensorValues[i] = (raw > sensorThreshold) ? 1 : 0;
    } else {
      sensorValues[i] = (raw > sensorThreshold) ? 0 : 1;
    }
  }
}

int calculatePosition() {
  int num = 0, den = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    num += sensorValues[i] * weights[i];
    den += sensorValues[i];
  }
  return (den == 0) ? 700 : (num / den);
}

void setMotorSpeed(int L, int R) {
    if (L >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else        { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); L = -L; }

    if (R >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else        { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); R = -R; }

    ledcWrite(pwmA, L);
    ledcWrite(pwmB, R);
}

void stopMotors() {
  ledcWrite(pwmA, 0);
  ledcWrite(pwmB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void spinLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(pwmA, 255);
  ledcWrite(pwmB, 255);
}

void spinRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  ledcWrite(pwmA, 255);
  ledcWrite(pwmB, 255);
}

void spinRightslow() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  ledcWrite(pwmA, 150);
  ledcWrite(pwmB, 150);
}
void spinLeftslow() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(pwmA, 150);
  ledcWrite(pwmB, 150);
}