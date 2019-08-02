int current_time, passed_time, last_time;

void print_time() {
  current_time = micros();
  passed_time = current_time - last_time;
  Serial.println(passed_time);
  Serial.println();
  last_time = current_time;
}
