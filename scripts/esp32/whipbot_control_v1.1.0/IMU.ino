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
    //    delay(10);
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

  aX = IMU.getAccelX_mss();
  aY = IMU.getAccelY_mss();
  aZ = -1 * IMU.getAccelZ_mss();

  mX = IMU.getMagX_uT();
  mY = IMU.getMagY_uT();
  mZ = IMU.getMagZ_uT();
  temperature_degC = IMU.getTemperature_C();
}


void complementary_filter() {
  ACCpitch = J * ACCpitch + (1 - J) * atan2(aY,  aZ);
  ACCroll = J * ACCroll + (1 - J) * atan2(aX, sqrt(pow(aY, 2) + pow(aZ, 2)));

  //calculate coefficients for complementary filter
  //remind aX, aY, aZ are in [m/s2], but g should be in [g], so you have to divide by 9.81 approx.
  g = sqrt(pow(aX, 2) + pow(aY, 2) + pow(aZ, 2)) / GRAVITATIONAL_ACCEL;
  k = 0.1 * pow(65536, -1 * (pow((1 - g), 2) / C));

  //calculate pitch/roll/yaw by gyrosensor (About pitch & roll, using Complementary Filter)
  pitch_data = (1 - k) * (pitch_data + (-1 * gX)  / SAMPLING_RATE) + k * ACCpitch; //ロールの角速度データが正負逆で入っているようだ。
  roll_data = (1 - k) * (roll_data + gY / SAMPLING_RATE) + k * ACCroll;

  if (abs(gZ) < (0.05 * M_PI / 180)) { //if gyro Z is lower than 0.05 deg/s (= 0.05*pi/180 rad)
    heading_data = heading_data;
  }
  else heading_data = heading_data + gZ / SAMPLING_RATE;

  if (heading_data > M_PI) {
    heading_data = -1 * 2 * M_PI + heading_data;
  }
  else if (heading_data < -1 * M_PI) {
    heading_data = 2 * M_PI + heading_data;
  }
}
