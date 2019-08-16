float ACC_roll, ACC_pitch; // roll and pitch based on accelerometer

//void initiate_gyro() {
//  for (int i = 0; i < SAMPLES_FOR_INITIATION; i++) {
//    //    delay(10);
//    IMU.readSensor();
//    sum_gX += IMU.getGyroX_rads();
//    sum_gY += IMU.getGyroY_rads();
//    sum_gZ += IMU.getGyroZ_rads();
//  }
//
//  offset_gX = sum_gX / SAMPLES_FOR_INITIATION;
//  offset_gY = sum_gY / SAMPLES_FOR_INITIATION;
//  offset_gZ = sum_gZ / SAMPLES_FOR_INITIATION;
//  Serial.println(offset_gX);
//}


void get_IMU_data(float *imu_data_func) {
  //  read IMU
  IMU.readSensor();

  imu_data_func[0] = IMU.getAccelX_mss();
  imu_data_func[1] = IMU.getAccelY_mss();
  imu_data_func[2] = -1 * IMU.getAccelZ_mss();

  //  contain sensor value to perpared parameters;
  //  translate gyro values to deg/s and subtract offset value
  imu_data_func[3] = IMU.getGyroX_rads();
  imu_data_func[4] = IMU.getGyroY_rads();
  imu_data_func[5] = IMU.getGyroZ_rads();

  imu_data_func[6] = IMU.getMagX_uT();
  imu_data_func[7] = IMU.getMagY_uT();
  imu_data_func[8] = IMU.getMagZ_uT();
  imu_data_func[9] = IMU.getTemperature_C();
}


void complementary_filter(float *posture_angle_func, float *imu_data_func) {
  //  coefficient of complementary filter
  float C = 1;  //  coefficient C
  float g;  // scale of accelereration
  float J = 0.8;  // coefficient of LPF on calculating posture angle based on accelerometer
  float k;  //  weight of filtering in complementary filter

  ACC_pitch = J * ACC_pitch + (1 - J) * atan2(imu_data_func[1],  imu_data_func[2]);
  ACC_roll = J * ACC_roll + (1 - J) * atan2(imu_data_func[0], sqrt(pow(imu_data_func[1], 2) + pow(imu_data_func[2], 2)));

  //calculate coefficients for complementary filter
  //remind aX, aY, aZ are in [m/s2], but g should be in [g], so you have to divide by 9.81 approx.
  g = sqrt(pow(imu_data_func[0], 2) + pow(imu_data_func[1], 2) + pow(imu_data_func[2], 2)) / GRAVITATIONAL_ACCEL;
  k = 0.1 * pow(65536, -1 * (pow((1 - g), 2) / C));

  //calculate pitch/roll by gyrosensor (About pitch & roll, using Complementary Filter)
  posture_angle_func[0] = (1 - k) * (posture_angle_func[0] + imu_data_func[4] / SAMPLING_RATE) + k * ACC_roll; //calculate roll
  posture_angle_func[1] = (1 - k) * (posture_angle_func[1] + (-1 * imu_data_func[3])  / SAMPLING_RATE) + k * ACC_pitch; //calculate pitch。

  //calculate heading angle by gyro (ignore small gyro data. assume them as its drift)
  if (abs(imu_data_func[5]) < (0.05 * M_PI / 180)) { //if gyro Z is lower than 0.05 deg/s (= 0.05*pi/180 rad)
    posture_angle_func[2] = posture_angle_func[2];
  }
  else posture_angle_func[2] = posture_angle_func[2] + imu_data_func[5] / SAMPLING_RATE;

  // convert range of heading angle to -180 ~ +180 deg
  if (posture_angle_func[2] > M_PI) {
    posture_angle_func[2] = -1 * 2 * M_PI + posture_angle_func[2];
  }
  else if (posture_angle_func[2] < -1 * M_PI) {
    posture_angle_func[2] = 2 * M_PI + posture_angle_func[2];
  }
}
