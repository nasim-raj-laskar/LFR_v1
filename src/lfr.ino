#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// --- Motor Driver Pins ---
#define MOTOR_R_PWM 13
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY  27
#define MOTOR_L_IN1 26
#define MOTOR_L_IN2 25
#define MOTOR_L_PWM 33

// --- PWM ---
#define PWM_FREQ 20000
#define PWM_RES  8

// --- MUX Pins ---
#define MUX_S0 18
#define MUX_S1 19
#define MUX_S2 17
#define MUX_S3 5
#define MUX_SIG 34

// --- OLED ---
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST -1
#define OLED_ADDR 0x3C

// --- Buttons ---
#define BTN_START 4
#define BTN_CAL   15
#define BTN_MODE  23
#define BTN_RESET 0

// --- Constants ---
#define NUM_SENSORS 16
#define SENSOR_MAX 4095
#define SENSOR_MIN 0

#define BASE_SPEED 120
#define MAX_SPEED 255

// ---- STABLE PID ----
#define PID_KP 0.18
#define PID_KI 0.0
#define PID_KD 2.5

#define EEPROM_SIZE 128
#define EEPROM_ADDR_MIN 0
#define EEPROM_ADDR_MAX 64

// --- Globals ---
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

int sensorValues[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

bool running = false;
bool whiteLine = false;

float lastError = 0;
float integral = 0;

// --- Prototypes ---
void readSensors();
int getLinePosition();
void calibrateSensors();
void setMotors(int left, int right);
void updateOLED();
void handleButtons();
void saveCalibration();
void loadCalibration();

void setup() {
  Serial.begin(115200);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  // ✅ ESP32 CORE v3 PWM
  ledcAttach(MOTOR_L_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_R_PWM, PWM_FREQ, PWM_RES);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_CAL, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();

  calibrateSensors();
  updateOLED();
}

void loop() {
  handleButtons();

  if (running) {
    readSensors();
    int position = getLinePosition();
    float error = position - (NUM_SENSORS - 1) * 500 / 2;

    integral += error;
    integral = constrain(integral, -5000, 5000);

    float derivative = error - lastError;
    float correction = PID_KP * error + PID_KI * integral + PID_KD * derivative;
    lastError = error;

    bool lostLine = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > 250) {
        lostLine = false;
        break;
      }
    }
    if (lostLine) {
      setMotors(0, 0);
      updateOLED();
      delay(50);
      return;
    }

    int leftPWM  = BASE_SPEED + correction;
    int rightPWM = BASE_SPEED - correction;

    leftPWM  = constrain(leftPWM,  0, MAX_SPEED);
    rightPWM = constrain(rightPWM, 0, MAX_SPEED);

    setMotors(leftPWM, rightPWM);
    updateOLED();
  } else {
    setMotors(0, 0);
  }

  delay(5);
}

// --- Sensor Reading ---
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(MUX_S0, i & 1);
    digitalWrite(MUX_S1, (i >> 1) & 1);
    digitalWrite(MUX_S2, (i >> 2) & 1);
    digitalWrite(MUX_S3, (i >> 3) & 1);
    delayMicroseconds(5);

    int val = analogRead(MUX_SIG);
    val = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    val = constrain(val, 0, 1000);
    if (whiteLine) val = 1000 - val;
    sensorValues[i] = val;
  }
}

// --- Line Position ---
int getLinePosition() {
  long avg = 0, sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    avg += (long)sensorValues[i] * i * 500;
    sum += sensorValues[i];
  }
  if (sum == 0) return (NUM_SENSORS - 1) * 500 / 2;
  return avg / sum;
}

// --- EEPROM ---
void saveCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.writeUShort(EEPROM_ADDR_MIN + i * 2, sensorMin[i]);
    EEPROM.writeUShort(EEPROM_ADDR_MAX + i * 2, sensorMax[i]);
  }
  EEPROM.commit();
}

void loadCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = EEPROM.readUShort(EEPROM_ADDR_MIN + i * 2);
    sensorMax[i] = EEPROM.readUShort(EEPROM_ADDR_MAX + i * 2);
    if (sensorMin[i] == 0xFFFF || sensorMax[i] == 0xFFFF) {
      sensorMin[i] = SENSOR_MAX;
      sensorMax[i] = SENSOR_MIN;
    }
  }
}

// --- Calibration ---
void calibrateSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = SENSOR_MAX;
    sensorMax[i] = SENSOR_MIN;
  }

  for (int t = 0; t < 400; t++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      digitalWrite(MUX_S0, i & 1);
      digitalWrite(MUX_S1, (i >> 1) & 1);
      digitalWrite(MUX_S2, (i >> 2) & 1);
      digitalWrite(MUX_S3, (i >> 3) & 1);
      delayMicroseconds(5);
      int val = analogRead(MUX_SIG);
      sensorMin[i] = min(sensorMin[i], val);
      sensorMax[i] = max(sensorMax[i], val);
    }
    delay(10);
  }
  saveCalibration();
}

// --- Motor Control ---
void setMotors(int left, int right) {
  digitalWrite(MOTOR_L_IN1, left > 0);
  digitalWrite(MOTOR_L_IN2, left <= 0);
  ledcWrite(MOTOR_L_PWM, abs(left));

  digitalWrite(MOTOR_R_IN1, right > 0);
  digitalWrite(MOTOR_R_IN2, right <= 0);
  ledcWrite(MOTOR_R_PWM, abs(right));
}

// --- OLED ---
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(running ? "RUN " : "IDLE ");
  display.print(whiteLine ? "WL" : "BL");

  for (int i = 0; i < NUM_SENSORS; i++) {
    int h = map(sensorValues[i], 0, 1000, 0, 40);
    display.fillRect(i * 8, 60 - h, 6, h, SSD1306_WHITE);
  }
  display.display();
}

// --- Buttons ---
void handleButtons() {
  static bool ls = HIGH, lc = HIGH, lm = HIGH, lr = HIGH;

  bool cs = digitalRead(BTN_START);
  bool cc = digitalRead(BTN_CAL);
  bool cm = digitalRead(BTN_MODE);
  bool cr = digitalRead(BTN_RESET);

  if (ls == HIGH && cs == LOW) running = !running;
  if (lc == HIGH && cc == LOW) calibrateSensors();
  if (lm == HIGH && cm == LOW) whiteLine = !whiteLine;
  if (lr == HIGH && cr == LOW) ESP.restart();

  ls = cs; lc = cc; lm = cm; lr = cr;
}
