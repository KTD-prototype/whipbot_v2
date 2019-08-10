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


void motor_drive(int pwm_output) {
  if (pwm_output >= 0) {
    ledcWrite(CHANNEL_L, pwm_output);
    ledcWrite(CHANNEL_R, pwm_output);

    digitalWrite(INA_L, HIGH);
    digitalWrite(INB_L, LOW);

    digitalWrite(INA_R, LOW);
    digitalWrite(INB_R, HIGH);
  }

  else {
    ledcWrite(CHANNEL_L, abs(pwm_output));
    ledcWrite(CHANNEL_R, abs(pwm_output));

    digitalWrite(INA_L, LOW);
    digitalWrite(INB_L, HIGH);

    digitalWrite(INA_R, HIGH);
    digitalWrite(INB_R, LOW);
  }
}
