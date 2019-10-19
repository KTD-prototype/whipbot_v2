//  reference:https://kougaku-navi.hatenablog.com/entry/20141008/p1
int receive_data() {
  byte high = Serial.read();
  byte low = Serial.read();
  int received_data = high * 256 + low;

  return received_data;
}

int receive_target() {
  byte high = Serial.read();
  byte low = Serial.read();
  int received_data = high * 256 + low;

  if (received_data > 1000) {
    received_data = received_data - 32000;
  }

  return received_data;
}
