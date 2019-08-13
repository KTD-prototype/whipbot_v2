bool motor_drive_enable() {
  // check battery voltage before drive motor
  int raw_voltage = analogRead(BATTERY_VOLTAGE_SENSE);
  voltage = raw_voltage * 3.3 * 4 / 4096;
  bool flag;

  if (voltage > 10.5) {
    flag = true;
  }
  else {
    flag = false;
  }

  // check robot's posture angle, if it is tilted too much, stop to drive motors.
  if (abs(posture_angle[1]) > (30.0 * M_PI / 180.0)) {
    flag = false;
  }

  return flag;
}


int pwm_limit(int pwm_output, int last_pwm_output) {
  if (abs(pwm_output - last_pwm_output) > 1500) {
    pwm_output = last_pwm_output;
  }

  int ramped_pwm_output = (pwm_output + last_pwm_output) / 2;

  if (ramped_pwm_output > 4095) {
    ramped_pwm_output = 4095;
  }
  else if (ramped_pwm_output < -4095) {
    ramped_pwm_output = -4095;
  }
  
  return ramped_pwm_output;
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
