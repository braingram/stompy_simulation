// read joystick, plot values

#include "SPI.h"
#include "ILI9341_t3.h"

#define JOY_X_PIN A0
#define JOY_Y_PIN A14
#define JOY_Z_PIN A10
#define DEADMAN_PIN 11  // requires internal pull-up

#define M1_EN_PIN 8
#define M2_EN_PIN 2

#define HIP_SENSOR_PIN A6
#define THIGH_SENSOR_PIN A3
#define KNEE_SENSOR_PIN A1
#define CALF_SENSOR_PIN A2

#define HIP_PWM_0 9
#define HIP_PWM_1 10
#define THIGH_PWM_0 3
#define THIGH_PWM_1 4
#define KNEE_PWM_0 5
#define KNEE_PWM_1 6

#define TFT_DC 21
#define TFT_CS 255
//#define TFT_RST 255  // 255 = unused, connect to 3.3V
#define TFT_RST 18
#define TFT_MOSI 7
#define TFT_SCLK 13
#define TFT_MISO 12


#define ANALOG_READ_RES 12
#define ANALOG_WRITE_RES 12
#define ANALOG_WRITE_FREQUENCY 2000


// for a 320 high display, scale analog values by
#define D_YSCALE 18
#define D_XSCALE 5
#define D_DEADMAN_X 200
#define D_DEADMAN_Y 40
#define D_DEADMAN_R 20
#define D_JOY_X 150
#define D_JOY_Y 40
#define D_JOY_R 20
#define D_JOY_ZR 10

ILI9341_t3 tft = ILI9341_t3(
  TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

unsigned int joy_raw[3] = {0, 0, 0};
float joy[3] = {0., 0., 0.};
#define JS_X 0
#define JS_Y 1
#define JS_Z 2
#define HIP_JS 2
#define THIGH_JS 1
#define KNEE_JS 0
#define N_JS 3
#define JOY_HALF 2047.5
// TODO dead region

unsigned int pwms[3] = {0, 0, 0};
// to go from 0,1 -> 0,max_pwm
// for 12 bit, and ~50% range = 2048
#define HIP_PWM_SCALE 1600
#define THIGH_PWM_SCALE 1600
#define KNEE_PWM_SCALE 1600

bool deadman = 0;

unsigned int display_x = 0;

unsigned int sensor_raw[4] = {0, 0, 0, 0};
unsigned int sensor_last[4] = {0, 0, 0, 0};

#define SENSOR_HIP 0
#define SENSOR_THIGH 1
#define SENSOR_KNEE 2
#define SENSOR_CALF 3 
#define N_SENSOR 4

#define DEBUG

elapsedMillis header_refresh;
elapsedMillis sensor_refresh;
elapsedMillis pwm_refresh;


void zero_pwms() {
  #ifdef DEBUG
  Serial.println("zero_pwms");
  #endif
  analogWrite(HIP_PWM_0, 0);
  analogWrite(HIP_PWM_1, 0);
  analogWrite(THIGH_PWM_0, 0);
  analogWrite(THIGH_PWM_1, 0);
  analogWrite(KNEE_PWM_0, 0);
  analogWrite(KNEE_PWM_1, 0);
}

void write_pwms() {
  if (joy[HIP_JS] > 0) {
    pwms[SENSOR_HIP] = joy[HIP_JS] * HIP_PWM_SCALE;
    analogWrite(HIP_PWM_0, pwms[SENSOR_HIP]);
    analogWrite(HIP_PWM_1, 0);
  } else {
    pwms[SENSOR_HIP] = -joy[HIP_JS] * HIP_PWM_SCALE;
    analogWrite(HIP_PWM_0, 0);
    analogWrite(HIP_PWM_1, pwms[SENSOR_HIP]);
  }
  if (joy[THIGH_JS] > 0) {
    pwms[SENSOR_THIGH] = joy[THIGH_JS] * THIGH_PWM_SCALE;
    analogWrite(THIGH_PWM_0, pwms[SENSOR_THIGH]);
    analogWrite(THIGH_PWM_1, 0);
  } else {
    pwms[SENSOR_THIGH] = -joy[THIGH_JS] * THIGH_PWM_SCALE;
    analogWrite(THIGH_PWM_0, 0);
    analogWrite(THIGH_PWM_1, pwms[SENSOR_THIGH]);
  }
  if (joy[KNEE_JS] > 0) {
    pwms[SENSOR_KNEE] = joy[KNEE_JS] * KNEE_PWM_SCALE;
    analogWrite(KNEE_PWM_0, pwms[SENSOR_KNEE]);
    analogWrite(KNEE_PWM_1, 0);
  } else {
    pwms[SENSOR_KNEE] = -joy[KNEE_JS] * KNEE_PWM_SCALE;
    analogWrite(KNEE_PWM_0, 0);
    analogWrite(KNEE_PWM_1, pwms[SENSOR_KNEE]);
  }
  #ifdef DEBUG
  Serial.println("write_pwms...");
  Serial.print("\tHIP: ");
  Serial.println(pwms[SENSOR_HIP]);
  Serial.print("\tTHIGH: ");
  Serial.println(pwms[SENSOR_THIGH]);
  Serial.print("\tKNEE: ");
  Serial.println(pwms[SENSOR_KNEE]);
  #endif
}

void disable_motors() {
  #ifdef DEBUG
  Serial.println("disable_motors");
  #endif
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  zero_pwms();
}

void enable_motors() {
  #ifdef DEBUG
  Serial.println("enable_motors");
  #endif
  digitalWrite(M1_EN_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);
}

void read_joystick() {
  deadman = digitalRead(DEADMAN_PIN);
  if (deadman) {
    disable_motors();
  } else {
    enable_motors();
  };
  joy_raw[JS_X] = analogRead(JOY_X_PIN);
  joy_raw[JS_Y] = analogRead(JOY_Y_PIN);
  joy_raw[JS_Z] = analogRead(JOY_Z_PIN);
  // TODO convert joystick raw to joy
  joy[JS_X] = (joy_raw[JS_X] / (float)JOY_HALF - 1.0);
  joy[JS_Y] = (joy_raw[JS_Y] / (float)JOY_HALF - 1.0);
  joy[JS_Z] = (joy_raw[JS_Z] / (float)JOY_HALF - 1.0);
  #ifdef DEBUG
  Serial.println("read_joystick...");
  Serial.print("\tX: ");
  Serial.print(joy_raw[JS_X]);
  Serial.print(" ");
  Serial.println(joy[JS_X], 2);
  Serial.print("\tY: ");
  Serial.print(joy_raw[JS_Y]);
  Serial.print(" ");
  Serial.println(joy[JS_Y], 2);
  Serial.print("\tZ: ");
  Serial.print(joy_raw[JS_Z]);
  Serial.print(" ");
  Serial.println(joy[JS_Z], 2);
  #endif
}

void read_sensors() {
  sensor_last[SENSOR_HIP] = sensor_raw[SENSOR_HIP];
  sensor_last[SENSOR_THIGH] = sensor_raw[SENSOR_THIGH];
  sensor_last[SENSOR_KNEE] = sensor_raw[SENSOR_KNEE];
  sensor_last[SENSOR_CALF] = sensor_raw[SENSOR_CALF];
  // TODO read 3
  sensor_raw[SENSOR_HIP] = analogRead(HIP_SENSOR_PIN);
  sensor_raw[SENSOR_THIGH] = analogRead(THIGH_SENSOR_PIN);
  sensor_raw[SENSOR_KNEE] = analogRead(KNEE_SENSOR_PIN);
  sensor_raw[SENSOR_CALF] = analogRead(CALF_SENSOR_PIN);
  #ifdef DEBUG
  Serial.println("read_sensors...");
  Serial.print("\tHIP: ");
  Serial.println(sensor_raw[SENSOR_HIP]);
  Serial.print("\tTHIGH: ");
  Serial.println(sensor_raw[SENSOR_THIGH]);
  Serial.print("\tKNEE: ");
  Serial.println(sensor_raw[SENSOR_KNEE]);
  Serial.print("\tCALF: ");
  Serial.println(sensor_raw[SENSOR_CALF]);
  #endif
}

void draw_values() {
  tft.fillRect(20, 0, 100, 80, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("H:");
  tft.println(sensor_raw[SENSOR_HIP]);
  tft.setTextColor(ILI9341_PINK);
  tft.print("T:");
  tft.println(sensor_raw[SENSOR_THIGH]);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("K:");
  tft.println(sensor_raw[SENSOR_KNEE]);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("C:");
  tft.println(sensor_raw[SENSOR_CALF]);
}

void draw_joystick() {
  tft.fillRect(D_JOY_X-D_JOY_R-D_JOY_ZR, D_JOY_Y-D_JOY_R-D_JOY_ZR, (D_JOY_R + D_JOY_ZR) * 2, (D_JOY_R + D_JOY_ZR) * 2, ILI9341_BLACK);
  // compute jx, jy, jr
  unsigned int jx = D_JOY_X - (joy[JS_X] * D_JOY_R);
  unsigned int jy = D_JOY_Y - (joy[JS_Y] * D_JOY_R);
  unsigned int jr = (joy[JS_Z] + 1.0) * 0.5 * D_JOY_R;
  /*
  unsigned int jx = D_JOY_X + (int)((joy_x / 2047.5 - 1.0) * D_JOY_R);
  unsigned int jy = D_JOY_Y + (int)((joy_y / 2047.5 - 1.0) * D_JOY_R);
  unsigned int jr = (unsigned int)((joy_z / 4095) * D_JOY_R);
  */
  /*
  Serial.print("JX:");
  Serial.println(jx);
  Serial.print("JY:");
  Serial.println(jy);
  Serial.print("JR:");
  Serial.println(jr);
  */
  tft.drawCircle(D_JOY_X, D_JOY_Y, D_JOY_R, ILI9341_WHITE);
  tft.drawLine(D_JOY_X, D_JOY_Y, jx, jy, ILI9341_GREEN);
  tft.fillCircle(jx, jy, jr, ILI9341_GREEN);
}

void draw_deadman() {
  // draw deadman
  if (deadman) {
    tft.fillCircle(D_DEADMAN_X, D_DEADMAN_Y, D_DEADMAN_R, ILI9341_BLACK);
    tft.drawCircle(D_DEADMAN_X, D_DEADMAN_Y, D_DEADMAN_R, ILI9341_GREEN); 
  } else {
    tft.fillCircle(D_DEADMAN_X, D_DEADMAN_Y, D_DEADMAN_R, ILI9341_PINK);
  }
}

void draw_plot() {
  unsigned int nx = display_x + D_XSCALE;
  if (display_x >= 240) {
    tft.fillRect(0, 80, 240, 240, ILI9341_BLACK);
    display_x = 0;
  } else {
    // draw lines front
    tft.drawLine(display_x, 320 - sensor_last[SENSOR_HIP] / D_YSCALE, nx, 320 - sensor_raw[SENSOR_HIP] / D_YSCALE, ILI9341_WHITE);
    tft.drawLine(display_x, 320 - sensor_last[SENSOR_THIGH] / D_YSCALE, nx, 320 - sensor_raw[SENSOR_THIGH] / D_YSCALE, ILI9341_PINK);
    tft.drawLine(display_x, 320 - sensor_last[SENSOR_KNEE] / D_YSCALE, nx, 320 - sensor_raw[SENSOR_KNEE] / D_YSCALE, ILI9341_GREEN);
    tft.drawLine(display_x, 320 - sensor_last[SENSOR_CALF] / D_YSCALE, nx, 320 - sensor_raw[SENSOR_CALF] / D_YSCALE, ILI9341_YELLOW);
    display_x = nx;
  }
}


void setup() {
  /*
  delay(1000);
  // HACK reset pin for display
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  delay(5);
  digitalWrite(18, LOW);
  delay(20);
  digitalWrite(18, HIGH);
  delay(150);
  */
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  pinMode(DEADMAN_PIN, INPUT_PULLUP);
  
  // disable motor outputs
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  disable_motors();
  
  // setup same analog read res
  analogReadResolution(ANALOG_READ_RES);
  analogWriteResolution(ANALOG_WRITE_RES);
  analogWriteFrequency(3, ANALOG_WRITE_FREQUENCY);
  analogWriteFrequency(5, ANALOG_WRITE_FREQUENCY);

  Serial.begin(115200);
}

void loop() {
  if (sensor_refresh > 100) {
    read_joystick();
    read_sensors();
    draw_plot();
    draw_joystick();
    draw_deadman();
    sensor_refresh = 0;
  }
  if (header_refresh > 500) {
    draw_values();
    header_refresh = 0;
  }
  if (pwm_refresh > 100) {
    if (!deadman) {
      write_pwms();
    } else {
      zero_pwms();
    };
    pwm_refresh = 0;
  }
}
