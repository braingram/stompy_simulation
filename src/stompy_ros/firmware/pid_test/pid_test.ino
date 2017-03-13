/*
 * Map input to 0 - 5 mm / s (to match cylinder 0 - 5 in / s)
 */

#define ENABLE_PIN 1
#define DIR_PIN 2
#define PWM_PIN 3
#define SENSOR_PIN A0
#define MAX_PWM_PERCENT 10
#define PWM_OFFSET 3276  // deadband
#define PWM_SCALE 131.07
#define CENTER_SPEED 6000

// raw sensor range is 0 - 16383, maps to 0 - 20 mm
#define SENSOR_SCALE 0.0012207776353537203
#define SENSOR_REPORT_PERIOD 1000

float velocity = 0;
float stroke = 0;
unsigned int last_update = 0;
float last_stroke = 0;

float target_velocity = 0;
float current_pwm = 0;
float pid_p = 20.0;
float pid_i = 0.0;
float pid_d = 0.0;
float pid_last_e = 0.0;
float pid_i_value = 0.0;
float pid_i_max = 100.0;
bool direct_output = false;

bool graphing = true;
int graph_line = 0;
bool graph_output = true;
elapsedMillis report_timer;

float sensor_to_stroke(int sensor_value) {
  // raw sensor range is 0 - 16383, maps to 0 - 20 mm
  return sensor_value * SENSOR_SCALE;
}

void update_velocity(float new_velocity) {
  velocity = 0.9 * velocity + 0.1 * new_velocity;
}

float read_sensor() {
  // read 3 readings, throw out high and low
  // map to 0-20 mm
  return sensor_to_stroke(middle_reading(
    analogRead(SENSOR_PIN),
    analogRead(SENSOR_PIN),
    analogRead(SENSOR_PIN)));
}

int middle_reading(int a0, int a1, int a2) {
  // take 3 values, return middle or most common value
  if (a0 == a1) return a0;
  if (a1 == a2) return a1;
  if (a0 == a2) return a2;
  if (a0 > a1) {
    if (a2 > a0) return a0;
    if (a1 > a2) {
      return a1;
    } else {
      return a2;
    }
  } else {
    if (a2 > a1) return a1;
    if (a0 > a2) {
      return a0;
    } else {
      return a2;
    }
  }
}

void setup() {
  analogWriteResolution(14);
  analogReadResolution(14);
  analogWriteFrequency(3, 2930);
  analogWriteFrequency(5, 2930);
  analogWriteFrequency(25, 2930);
  // enable motor controllers
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  //digitalWrite(ENABLE_PIN, HIGH);
  stroke = read_sensor();
  set_pwm_direct(0);
}

void set_pwm(float value) {
  // input is 0-100, output should be 0-16383
  //Serial.println(value);
  if (value == 0) {
    set_pwm_direct(0.0);
    return;
  }
  if (value > 0) {
    set_pwm_direct(value * PWM_SCALE + PWM_OFFSET);
    /*
    digitalWrite(DIR_PIN, 0);
    analogWrite(PWM_PIN, value * PWM_SCALE + PWM_OFFSET);
    */
  } else {
    set_pwm_direct(value * PWM_SCALE + -PWM_OFFSET);
    /*
    digitalWrite(DIR_PIN, 1);
    analogWrite(PWM_PIN, value * -PWM_SCALE + PWM_OFFSET);
    */
  }
}

void set_pwm_direct(float value) {
  //Serial.println(value);
  if (value > 0) {
    digitalWrite(DIR_PIN, 0);
    analogWrite(PWM_PIN, value);
  } else {
    digitalWrite(DIR_PIN, 1);
    analogWrite(PWM_PIN, -value);
  }
}

void update_serial() {
  if (Serial.available()) {
    char c = Serial.read();
    float dv;
    switch (c) {
      case '?':
        // print out current pid
        Serial.print("?P ");
        Serial.print(pid_p);
        Serial.print(" I ");
        Serial.print(pid_i);
        Serial.print(" D ");
        Serial.print(pid_d);
        Serial.print(" IM ");
        Serial.println(pid_i_max);
        break;
      case 'c':
        center();
        Serial.println("?center ");
        break;
      case 'e':
        digitalWrite(ENABLE_PIN, HIGH);
        Serial.println("?enable ");
        break;
      case 'd':
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println("?disable ");
        break;
      case 'v':
        target_velocity = Serial.parseFloat();
        current_pwm = 10. * target_velocity;
        graphing = true;
        direct_output = false;
        Serial.print("?target_velocity ");
        Serial.println(target_velocity);
        Serial.print("?curret_pwm ");
        Serial.println(current_pwm);
        break;
      case 'P':
        pid_p = Serial.parseFloat();
        Serial.print("?pid_p ");
        Serial.println(pid_p);
        break;
      case 'I':
        pid_i = Serial.parseFloat();
        Serial.print("?pid_i ");
        Serial.println(pid_i);
        break;
      case 'D':
        pid_d = Serial.parseFloat();
        Serial.print("?pid_d ");
        Serial.println(pid_d);
        break;
      case 'M':
        pid_i_max = Serial.parseFloat();
        Serial.print("?pid_i_max ");
        Serial.println(pid_i_max);
        break;
      case 'o':
        dv = Serial.parseFloat();
        if (dv == 0) {
          set_pwm_direct(0.0);
          direct_output = false;
        } else {
          set_pwm_direct(dv);
          graphing = true;
          direct_output = true;
        }
        break;
      case 'g':
        graph_output = !graph_output;
        break;
      case 'G':
        graphing = !graphing;
        graph_line = 0;
        break;
    }
  }
}

void center() {
  digitalWrite(ENABLE_PIN, HIGH);
  float sv = read_sensor();
  while (sv < 10.) {
    set_pwm_direct(CENTER_SPEED);
    sv = read_sensor();
    if ((sv < 1) || (sv > 19)) {
      set_pwm_direct(0);
      return;
    }
  }
  set_pwm_direct(0);
  while (sv > 10.) {
    set_pwm_direct(-CENTER_SPEED);
    sv = read_sensor();
    if ((sv < 1) || (sv > 19)) {
      set_pwm_direct(0);
      return;
    }
  }
  set_pwm_direct(0);
  digitalWrite(ENABLE_PIN, LOW);
  target_velocity = 0.;
  current_pwm = 0;
  digitalWrite(ENABLE_PIN, HIGH);
  graphing = false;
  graph_line = 0;
}

void loop() {
  // store last sensor reading
  last_stroke = stroke;
  // update sensor
  unsigned int update_time = micros();
  float dt = (update_time - last_update) / 1000000.;
  stroke = read_sensor();
  //Serial.println(stroke, 6);
  // check limits, turn off if outside limits
  if ((stroke < 3) || (stroke > 17)) {
    digitalWrite(ENABLE_PIN, LOW);
    graphing = false;
    graph_line = 0;
  }
  // update velocity
  float new_velocity = (stroke - last_stroke) / dt;
  //Serial.println(new_velocity, 6);
  update_velocity(new_velocity);
  //Serial.println(velocity, 6);
  last_update = update_time;
  
  // run pid
  // compute error
  float pid_e = target_velocity - velocity;
  pid_i_value += pid_e;
  if (pid_i_value > pid_i_max) {
    pid_i_value = pid_i_max;
  } else if (pid_i_value < -pid_i_max) {
    pid_i_value = -pid_i_max;
  };
  float p = pid_e * pid_p
      + pid_i * pid_i_value
      + pid_d * ((pid_e - pid_last_e) / dt);
  pid_last_e = pid_e;
  float new_pwm = current_pwm + p;
  // set pwm to output, should go to 0 when error goes to 0
  if (current_pwm == 0) {
    new_pwm = 0;
  } else {
    if (current_pwm > 0) {
      // make sure new_pwm is between 0 and 100
      new_pwm = max(0, min(new_pwm, 100.));
    } else {
      new_pwm = -max(0, min(-new_pwm, 100.));
    }
  }
  if (!direct_output) set_pwm(new_pwm);

  update_serial();
  
  // get serial input
  if (!graph_output) {
    if (report_timer > SENSOR_REPORT_PERIOD) {
      // output information
      /*
      Serial.print("T");
      Serial.println(update_time);
      Serial.print("n");
      Serial.println(new_velocity);
      */
      Serial.print("V");
      Serial.println(velocity);
      Serial.print("S");
      Serial.println(stroke);
      Serial.print("e");
      Serial.println(pid_e);
      Serial.print("p");
      Serial.println(p);
      Serial.print("n");
      Serial.println(new_pwm);
      Serial.println("");
      report_timer = 0;
    }
  } else {
    if (graphing) {
      // draw output graph: new_pwm, velocity, stroke
      char line[41];
      // velocity: -10, 10
      // stroke: 0, 20
      // new_pwm: -100, 100
      // pid error (pid_e) -100:100?
      byte vg = (velocity + 10) * 2;
      byte sg = stroke * 2;
      byte pg = (new_pwm + 100) / 5.;
      byte eg = (pid_e * 10 + 20);
      for (byte i; i<40; i++) {
        line[i] = ' ';
        if (i == 19) line[i] = '|';
        if (i == vg) line[i] = 'v';
        if (i == sg) line[i] = 's';
        if (i == pg) line[i] = 'p';
        if (i == eg) line[i] = 'e';
      }
      line[40] = '\x00';
      Serial.print(line);
      Serial.print(':');
      Serial.print(abs(pid_e));
      Serial.print(':');
      Serial.println(graph_line);
      
      graph_line += 1;
    }
  }
  delay(10);  // throttle
}
