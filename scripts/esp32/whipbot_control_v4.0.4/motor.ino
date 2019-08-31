bool motor_drive_enable(float voltage) {
  bool flag;

  // check battery voltage
  //  if (voltage > 10.5) {
  //    flag = true;
  //  }
  //  else {
  //    flag = false;
  //  }

  // check robot's posture angle, if it is tilted too much, stop to drive motors.
  if (abs(posture_angle[1]) > (30.0 * M_PI / 180.0)) {
    flag = false;
  }
  else{
    flag = true;
  }

  return flag;
}


int ramp_pwm(int pwm_output, int last_pwm_output) {
  int ramp_factor = 1000;

  if (pwm_output > last_pwm_output + ramp_factor) {
    pwm_output = last_pwm_output + ramp_factor;
  }
  else if (pwm_output < last_pwm_output - ramp_factor) {
    pwm_output = last_pwm_output - ramp_factor;
  }

  if (pwm_output > 4095) {
    pwm_output = 4095;
  }
  else if (pwm_output < -4095) {
    pwm_output = -4095;
  }

  return pwm_output;
}


void motor_drive_L(int pwm_L) {
  if (pwm_L >= 0) {
    ledcWrite(CHANNEL_L, pwm_L);

    digitalWrite(INA_L, HIGH);
    digitalWrite(INB_L, LOW);
  }
  else {
    ledcWrite(CHANNEL_L, abs(pwm_L));

    digitalWrite(INA_L, LOW);
    digitalWrite(INB_L, HIGH);
  }
}

void motor_drive_R(int pwm_R) {
  if (pwm_R >= 0) {
    ledcWrite(CHANNEL_R, pwm_R);

    digitalWrite(INA_R, LOW);
    digitalWrite(INB_R, HIGH);
  }
  else {
    ledcWrite(CHANNEL_R, abs(pwm_R));

    digitalWrite(INA_R, HIGH);
    digitalWrite(INB_R, LOW);
  }
}
