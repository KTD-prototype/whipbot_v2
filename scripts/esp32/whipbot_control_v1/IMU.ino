//  coefficient of complementary filter
//  coefficient 1
#define C 1
//  scolar of the total acceleration
float g;
//  weight of filtering posture value via accelerometer
#define J 0.5
//  weight of filtering in complementary filter
float k;


void initiate_gyro() {
  for (int i = 0; i < SAMPLES_FOR_INITIATION; i++) {
    delay(10);
    IMU.readSensor();
    sum_gX = IMU.getGyroX_rads();
    sum_gY = IMU.getGyroY_rads();
    sum_gZ = IMU.getGyroZ_rads();
  }

  offset_gX = sum_gX / SAMPLES_FOR_INITIATION;
  offset_gY = sum_gY / SAMPLES_FOR_INITIATION;
  offset_gZ = sum_gZ / SAMPLES_FOR_INITIATION;
  Serial.println(offset_gX);
}


void get_IMU_data() {
  //  read IMU
  IMU.readSensor();

  //  contain sensor value to perpared parameters;
  //  translate gyro values to deg/s and subtract offset value
  gX = IMU.getGyroX_rads() - offset_gX;
  gY = IMU.getGyroY_rads() - offset_gY;
  gZ = IMU.getGyroZ_rads() - offset_gZ;

  //  translate accel calues to g
  aX = IMU.getAccelX_mss() / GRAVITATIONAL_ACCEL;
  aY = IMU.getAccelY_mss() / GRAVITATIONAL_ACCEL;
  aZ = -1 * IMU.getAccelZ_mss() / GRAVITATIONAL_ACCEL;

  mX = IMU.getMagX_uT();
  mY = IMU.getMagY_uT();
  mZ = IMU.getMagZ_uT();
  temperature_degC = IMU.getTemperature_C();
}


void complementary_filter() {
  ACCroll = J * ACCroll + (1 - J) * atan2(aY,  aZ);
  ACCpitch = J * ACCpitch + (1 - J) * atan2(aX, sqrt(pow(aY, 2) + pow(aZ, 2)));

  //calculate pitch/roll/yaw by gyrosensor (About pitch & roll, using Complementary Filter)
  g = sqrt(pow(aX, 2) + pow(aY, 2) + pow(aZ, 2));
  k = 0.1 * pow(65536, -1 * (pow((1 - g), 2) / C));
  roll = (1 - k) * (roll + (-1 * gX)  / SAMPLING_RATE) + k * ACCroll; //ロールの角速度データが正負逆で入っているようだ。
  pitch = (1 - k) * (pitch + gY / SAMPLING_RATE) + k * ACCpitch;

  mod_roll = pitch;
  mod_pitch = roll;

  if (abs(gZ) < 0.01) {
    heading = heading;
  }
  else heading = heading + gZ / SAMPLING_RATE;

  if (heading > M_PI) {
    heading = -1 * 2 * M_PI + heading;
  }
  else if (heading < -1 * M_PI) {
    heading = 2 * M_PI + heading;
  }
}
