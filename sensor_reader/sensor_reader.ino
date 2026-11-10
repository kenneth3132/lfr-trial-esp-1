/*
 * ======================================================================================
 * SENSOR READER & CALIBRATION TOOL for ESP32 + SmartElex RLS-08
 * ======================================================================================
 * Hardware:
 *   - Microcontroller : ESP32 Dev Module (on COM9)
 *   - Line Sensor     : SmartElex RLS-08 (8 IR Sensors directly to ESP32 ADC)
 *   - User Interface  : 3 Push Buttons + Built-in LED (GPIO 2)
 * 
 * Button Controls:
 *   - Button 1 (GPIO 27) : Toggle continuous live sensor monitor on Serial Monitor.
 *   - Button 2 (GPIO 14) : CALIBRATE track surface (samples white & black to set thresholds).
 *   - Button 3 (GPIO 13) : TOGGLE Track Mode (Black Line on White vs White Line on Black).
 * 
 * Pinout Mapping (SmartElex RLS-08 A1-A8 to ESP32 ADC):
 *   - Sensor 1 (Far Left)  : GPIO 36 (VP)  - ADC1_CH0
 *   - Sensor 2             : GPIO 39 (VN)  - ADC1_CH3
 *   - Sensor 3             : GPIO 34       - ADC1_CH6
 *   - Sensor 4 (Mid Left)  : GPIO 35       - ADC1_CH7
 *   - Sensor 5 (Mid Right) : GPIO 32       - ADC1_CH4
 *   - Sensor 6             : GPIO 33       - ADC1_CH5
 *   - Sensor 7             : GPIO 25       - ADC2_CH8
 *   - Sensor 8 (Far Right) : GPIO 26       - ADC2_CH9
 * ======================================================================================
 */

// ================= PIN DEFINITIONS =================
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// Push Buttons (Internal Pull-Up: Pressed = LOW, Released = HIGH)
const int BTN_START = 27;  // Button 1: Start / Pause monitor
const int BTN_CALIB = 14;  // Button 2: Calibrate on track surface
const int BTN_MODE  = 13;  // Button 3: Toggle Black / White line mode

// Onboard Status LED
const int LED_PIN = 2;

// ================= CALIBRATION & SENSOR ARRAYS =================
// Raw ADC reading values (0 to 4095 on ESP32 12-bit ADC)
int rawValues[NUM_SENSORS];

// Normalized values (0 = surface, 1000 = line)
int normValues[NUM_SENSORS];

// Binary values (0 = background, 1 = line detected)
int binaryValues[NUM_SENSORS];

// Calibration minimums (recorded over white surface)
int minValues[NUM_SENSORS];

// Calibration maximums (recorded over black line)
int maxValues[NUM_SENSORS];

// Dynamic midpoints for line thresholding
int thresholds[NUM_SENSORS];

// Track Mode: 0 = Black Line on White Surface, 1 = White Line on Black Surface
int trackMode = 0;

// Sensor Weights for Weighted Centroid Calculation
// Center of robot is 0. Far left is -3500, far right is +3500.
const int weights[NUM_SENSORS] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

bool isCalibrated = false;
bool liveMonitorActive = true;

// ================= DEBOUNCE TIMERS =================
unsigned long lastBtnCheck = 0;
const unsigned long debounceTime = 250;

// ================= FUNCTION DECLARATIONS =================
void readRawSensors();
void calibrateSurface();
int calculateLinePosition();
void printSensorReport();
void flashLED(int times, int delayMs);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n========================================================"));
  Serial.println(F("   ESP32 + SmartElex RLS-08 Sensor Calibration Tool    "));
  Serial.println(F("========================================================"));
  Serial.println(F("Button 1 (GPIO 27) : Pause/Resume Serial Stream"));
  Serial.println(F("Button 2 (GPIO 14) : CALIBRATE (Sweep sensors across line)"));
  Serial.println(F("Button 3 (GPIO 13) : Toggle Black/White Track Mode"));
  Serial.println(F("========================================================\n"));

  // Configure Button pins with internal pull-up resistors
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);

  // Configure Onboard LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize calibration arrays with safe default values
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 4095; // Will be driven down during calibration
    maxValues[i] = 0;    // Will be driven up during calibration
    thresholds[i] = 2000;// Midpoint default
  }

  flashLED(2, 100);
}

void loop() {
  unsigned long now = millis();

  // ----------------------------------------------------
  // 1. BUTTON 1: Pause / Resume Real-Time Serial Stream
  // ----------------------------------------------------
  if (digitalRead(BTN_START) == LOW && (now - lastBtnCheck > debounceTime)) {
    lastBtnCheck = now;
    liveMonitorActive = !liveMonitorActive;
    Serial.print(F("[INFO] Live monitor "));
    Serial.println(liveMonitorActive ? F("RESUMED.") : F("PAUSED."));
    flashLED(1, 100);
  }

  // ----------------------------------------------------
  // 2. BUTTON 2: CALIBRATE SENSORS ON TRACK SURFACE
  // ----------------------------------------------------
  if (digitalRead(BTN_CALIB) == LOW && (now - lastBtnCheck > debounceTime)) {
    lastBtnCheck = now;
    calibrateSurface();
  }

  // ----------------------------------------------------
  // 3. BUTTON 3: TOGGLE BLACK / WHITE TRACK MODE
  // ----------------------------------------------------
  if (digitalRead(BTN_MODE) == LOW && (now - lastBtnCheck > debounceTime)) {
    lastBtnCheck = now;
    trackMode = (trackMode == 0) ? 1 : 0;
    Serial.print(F("[MODE CHANGED] Current Track Mode: "));
    if (trackMode == 0) {
      Serial.println(F("BLACK line on White surface"));
      flashLED(1, 300);
    } else {
      Serial.println(F("WHITE line on Black surface"));
      flashLED(2, 150);
    }
  }

  // ----------------------------------------------------
  // 4. PERIODIC SENSOR READING & STREAMING
  // ----------------------------------------------------
  static unsigned long lastPrintTime = 0;
  if (liveMonitorActive && (now - lastPrintTime >= 100)) { // 10 Hz refresh
    lastPrintTime = now;
    readRawSensors();
    printSensorReport();
  }
}

// ======================================================================================
// SENSOR READING EXPLANATION:
// 1. readRawSensors():
//    - Reads each ADC channel (0 to 4095).
//    - High voltage (~3000-4000) corresponds to low reflection (Black surface / line).
//    - Low voltage (~200-800) corresponds to high reflection (White background).
// 2. Normalization:
//    - Maps each sensor between 0 (background) and 1000 (line) using calibrated min & max.
//    - This ensures all 8 sensors have identical sensitivity even if components vary.
// 3. Binary classification:
//    - Compares raw value against calculated threshold.
// ======================================================================================
void readRawSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Read raw ADC value from ESP32 pin
    rawValues[i] = analogRead(sensorPins[i]);

    // If calibrated, normalize the value between 0 and 1000
    if (isCalibrated && (maxValues[i] > minValues[i])) {
      int norm = map(rawValues[i], minValues[i], maxValues[i], 0, 1000);
      normValues[i] = constrain(norm, 0, 1000);
    } else {
      normValues[i] = rawValues[i];
    }

    // Determine binary state (0 = background, 1 = line) based on trackMode
    if (trackMode == 0) {
      // Black line: Raw reading is HIGHER than threshold
      binaryValues[i] = (rawValues[i] > thresholds[i]) ? 1 : 0;
    } else {
      // White line: Raw reading is LOWER than threshold
      binaryValues[i] = (rawValues[i] < thresholds[i]) ? 1 : 0;
    }
  }
}

// ======================================================================================
// CALIBRATION ROUTINE:
// When Button 2 is pressed, this function runs for ~4 seconds.
// You simply slide/sweep the robot slowly left and right across the line so that
// every one of the 8 sensors passes over BOTH the white surface and the black line.
// The algorithm records the lowest reading (white) and highest reading (black) for each
// individual sensor, and sets the threshold exactly at the midpoint!
// ======================================================================================
void calibrateSurface() {
  Serial.println(F("\n>>> CALIBRATION STARTED! <<<"));
  Serial.println(F("Slide the robot slowly left and right across the line..."));
  
  // Reset min/max limits
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  digitalWrite(LED_PIN, HIGH);
  unsigned long startCalib = millis();

  // Sample continuously for 4000 milliseconds (4 seconds)
  while (millis() - startCalib < 4000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int reading = analogRead(sensorPins[i]);
      if (reading < minValues[i]) minValues[i] = reading;
      if (reading > maxValues[i]) maxValues[i] = reading;
    }
    // Blink LED rapidly to indicate calibration in progress
    digitalWrite(LED_PIN, (millis() / 100) % 2);
    delay(5);
  }

  // Calculate dynamic midpoint thresholds for each sensor
  for (int i = 0; i < NUM_SENSORS; i++) {
    thresholds[i] = (minValues[i] + maxValues[i]) / 2;
  }

  isCalibrated = true;
  digitalWrite(LED_PIN, LOW);

  Serial.println(F(">>> CALIBRATION COMPLETE! <<<"));
  Serial.println(F("Sensor | Min(White) | Max(Black) | Threshold"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("  S")); Serial.print(i + 1);
    Serial.print(F("   |    ")); Serial.print(minValues[i]);
    Serial.print(F("    |    ")); Serial.print(maxValues[i]);
    Serial.print(F("    |    ")); Serial.println(thresholds[i]);
  }
  Serial.println(F("========================================================\n"));
  flashLED(3, 100);
}

// ======================================================================================
// WEIGHTED CENTROID CALCULATION:
// Computes line position relative to the center of the sensor array.
// Returns a value from -3500 (far left) to +3500 (far right).
// 0 means the line is dead-center between Sensor 4 and Sensor 5.
// ======================================================================================
int calculateLinePosition() {
  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // We use normalized analog values (0 to 1000) for sub-millimeter precision
    int val = (trackMode == 0) ? normValues[i] : (1000 - normValues[i]);
    
    // Only consider sensors reading above noise floor (e.g. > 150)
    if (val > 150) {
      numerator += (long)val * weights[i];
      denominator += val;
    }
  }

  // If no sensor detects the line (robot is completely off track)
  if (denominator == 0) {
    return 9999; // Special code for "Line Lost"
  }

  return (int)(numerator / denominator);
}

// ======================================================================================
// SERIAL MONITOR REPORT:
// Formats raw ADC readings, binary visual track bar, and position error.
// ======================================================================================
void printSensorReport() {
  int pos = calculateLinePosition();

  // Print binary visualization: e.g. [ .  .  .  #  #  .  .  . ]
  Serial.print(F("[ "));
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (binaryValues[i] == 1) {
      Serial.print(F("# ")); // Line detected
    } else {
      Serial.print(F(". ")); // Background
    }
  }
  Serial.print(F("] | "));

  // Print calculated position
  if (pos == 9999) {
    Serial.print(F("Pos: LOST     | "));
  } else {
    Serial.print(F("Pos: "));
    if (pos >= 0) Serial.print(F("+"));
    Serial.print(pos);
    Serial.print(F("      | "));
  }

  // Print Track mode
  Serial.print(trackMode == 0 ? F("Mode: BLACK") : F("Mode: WHITE"));
  Serial.println();
}

// Helper utility to blink LED
void flashLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(delayMs);
  }
}
