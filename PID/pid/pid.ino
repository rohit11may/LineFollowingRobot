const int sensorPins[] = {A0, A1, A2, A3, A4, A5}; // pin that the sensor is attached to

int sensorValue = 0;         // the sensor value
int sensorMin = 0;        // minimum sensor value
int sensorMax = 1023;           // maximum sensor value
int sensor[] = {0, 0, 0, 0, 0, 0};

int motor_left[] = {11, 10};
int motor_right[] = {3, 5};

int line_sum = 0;
bool no_line = true;
int pinCount = 6;

float Kp = 355, Ki = 0.0002, Kd = 7000;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int initial_motor_speed = 255;

void setup() {
  // turn on LED to signal the start of the calibration period:
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  Serial.begin(19200);
  // calibrate during the first five seconds
  while (millis() < 10000) {
    for (int thisPin = 0; thisPin < pinCount; thisPin++) {
      sensorValue = analogRead(sensorPins[thisPin]);
      //      Serial.println(sensorValue);
      // record the maximum sensor value
      if (sensorValue > sensorMax) {
        sensorMax = sensorValue;
      }

      // record the minimum sensor value
      if (sensorValue < sensorMin) {
        sensorMin = sensorValue;
      }
    }
  }
  // signal the end of the calibration period
  digitalWrite(12, LOW);
}

void loop() {

  read_sensors();
  calculate_pid();
  motor_control();

}





void calculate_pid() {
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
}

void read_sensors() {
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    sensorValue = analogRead(sensorPins[thisPin]);
    // apply the calibration to the sensor reading
    sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
    // in case the sensor value is outside the range seen during calibration
    sensorValue = constrain(sensorValue, 0, 255);
    sensor[thisPin] = sensorValue;
//    Serial.print(sensorValue + '\t');
//    Serial.print('\t');  
  }

  line_sum = 0;
  error = 0;
  no_line = true;
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    if (sensor[thisPin] > 100 && no_line == true) {
      no_line = false;
    }

    error = error + (sensor[thisPin] * (thisPin + 1));
    line_sum = line_sum + sensor[thisPin];
  }
  
  error = (error / line_sum) - 3;
  
  if (no_line == true){
    if (previous_error  > 0) {
      error = 3;
    } else {
      error = -3;
    }
  }
  
   
//  Serial.print(error);
//  Serial.println();
}

void motor_control() {
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  constrain(left_motor_speed, 0, 255);
  constrain(right_motor_speed, 0, 255);

  drive(left_motor_speed, right_motor_speed);
}

void mstop() {
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);

  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
  delay(25);
}

void drive(int mL, int mR) {
  analogWrite(motor_left[0], mL);
  digitalWrite(motor_left[1], LOW);

  analogWrite(motor_right[0], mR);
  digitalWrite(motor_right[1], LOW);
}
