byte process_byte(byte pulse) {
  if (pulse == 3) {
    pulse = 2;
  }
  else if (pulse == 2) {
    pulse = 3;
  }
  return pulse;
}


long count_encoder(byte pulse, byte last_pulse, long count) {
  //  exception process
  if (pulse == last_pulse) {
    //  do nothing when encoder signal has chattered.
  }
  else if (pulse == 0 && last_pulse == 3) {
    count--;
  }
  else if (pulse == 3 && last_pulse == 0) {
    count++;
  }

  //  counting encoder;main process
  else if (abs(pulse - last_pulse) > 1) {
    // do nothing when encoder signal has hopped(= chattering).
  }

  else if (pulse > last_pulse) {
    count--;
  }
  else {
    count++;
  }

  return count;
}
