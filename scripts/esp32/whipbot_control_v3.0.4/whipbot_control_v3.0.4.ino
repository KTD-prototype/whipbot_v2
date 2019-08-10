#include "MPU9250.h"

// use pin IO17 & IO16 for input of the encoder of the left moter, IO34 & IO35 for right motor
#define ENC_L_A 17
#define ENC_L_B 16
#define ENC_R_A 35
#define ENC_R_B 34

// use pin IO1 & IO3 as output of LEFT motor driver INA & INB, pin IO22 & IO0 for RIGHT one
#define INA_L 15
#define INB_L 14
#define INA_R 22
#define INB_R 0

// use pin IO4(A10) as PWM output for LEFT motor driver, pin 2 (A12) as RIGHT one
#define PWM_L A10
#define PWM_R A12
// channels for PWM
#define CHANNEL_L 0
#define CHANNEL_R 1

// use pin 36 (A0) as analog read for LEFT motor driver(current sense), pin 39(A3) for RIGHT
#define CURRENT_SENSE_L A0
#define CURRENT_SENSE_R A3

//use pin 25 as disabling LEFT motor driver, pin 26 for RIGHT one.
#define DISABLE_L 25
#define DISABLE_R 26

//use pin 27(A17) as analog read for battery voltage
#define BATTERY_VOLTAGE_SENSE A17

// configuration of servo city motor : https://www.servocity.com/317-rpm-spur-gear-motor-w-encoder
#define PULSE_PER_ROUND 723.24  //  at output shaft

// parameters to get IMU information
#define GRAVITATIONAL_ACCEL 9.798 //at TOKYO
#define SAMPLING_RATE 100 //IMU referesh rate : 200 Hz (over 1000 Hz is available if you want only refreshing) 
#define SAMPLES_FOR_INITIATION 200 // (not used) get mean value of 200 samples of gyro to cancel native bias
// channel number for timer interruption
#define INTERRUPTION_CHANNEL 0

// use pin 27 for Chip Select of SPI communication
#define SPI_CHIP_SELECT 21

// gain of velocity control
#define VELOCITY_CONTROL_GAIN_LINEAR 1
#define VELOCITY_CONTROL_GAIN_ANGULAR 1

// flag to communicate with host PC (if false, just print to serial monitor)
#define COM_FLAG true


// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI, SPI_CHIP_SELECT);
int status;


//  parameters for interrupt process
volatile byte pulse_L, last_pulse_L, pulse_R, last_pulse_R;
volatile long encoder_count_L = 0, encoder_count_R = 0;
//volatile bool LED = false; // for debug

//  parameters for recording time per loop
int time1, time2, dt;
int encoder_count_for_output = 0;

//  parameters for motor output shaft state
long current_encoder_count_L, last_encoder_count_L = 0, current_encoder_count_R = 0, last_encoder_count_R = 0;
float total_round_L = 0, last_round_L = 0, rpm_L = 0, total_round_R = 0, last_round_R = 0, rpm_R = 0;

//  parameters for timer interruption
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// parameter to trigger imu read process
volatile bool imu_flag = false;
// parameters for imu data
float imu_data[10]; //accel XYZ, gyro XYZ, mag XYZ, temperature
float posture_angle[3]; //roll, pitch, heading

// parameters for porsture control of the robot
int Kp_posture = 0, Ki_posture = 0, Kd_posture = 0; //PID gains will modified through command from host PC
float target_angle = 0.08;
float accumulated_angle_error = 0;

// velocity command from host PC. Unit is [cm/sec], [*0.01 rad/sec] when gain is 1
float velocity_command_linear = 0, velocity_command_angular = 0;

int current_time, passed_time, last_time, count = 0;



// function that will be called when GPIO interruption was fired
void IRAM_ATTR ISR() {
  //  refresh each value if it is changed
  //  handle those as a single 2 bit number; 1st bit is High/Low of A, 2nd bit is High/Low of B
  //  e.g.1: if pulse A is HIGH, pulse value is 1. Shift it to left, then you get 10. if pulse B is also high, add it to the 10, then you get 11, as a combined information of the A and B
  //  e.g.2: if pulse A is LOW, pulse value is 0. Shift it to left, then you get 00. if pulse B is high, add it to the 00, then you get 01, as a combined information of the A and B
  pulse_L = !digitalRead(ENC_L_A) << 1;
  pulse_L += !digitalRead(ENC_L_B);
  pulse_R = !digitalRead(ENC_R_A) << 1;
  pulse_R += !digitalRead(ENC_R_B);

  //  pulse_L will incremented as 0, 01(1), 11(3), 10(2) or reversed order of them.
  //  exchange value of 2 & 3 since this order is a bit confusing.
  pulse_L = process_byte(pulse_L);
  pulse_R = process_byte(pulse_R);

  // count up encoder based on output pulse
  encoder_count_L = count_encoder(pulse_L, last_pulse_L, encoder_count_L, 'L');
  encoder_count_R = count_encoder(pulse_R, last_pulse_R, encoder_count_R, 'R');

  //  reserve pulse info as last one of those
  last_pulse_L = pulse_L;
  last_pulse_R = pulse_R;
}


// function that will be called when timer interruption was fired
void IRAM_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);

  //  turn on the flag to read IMU
  imu_flag = true;

  isrCounter++;
  lastIsrAt = 0;
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // It is safe to use digitalRead/Write here if you want to toggle an output
}


void setup() {
  Serial.begin(115200);
  delay(10);


  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(INTERRUPTION_CHANNEL, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000 / SAMPLING_RATE, true);

  // Start an alarm
  timerAlarmEnable(timer);

  while (!Serial) {}
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  //  initiating process was included in the imported library, so following process are not needed
  //  initiate_gyro();


  // setting pin mode for encoder
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // setting pin mode for motor driver
  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  ledcSetup(CHANNEL_L, 12800, 10); // channel:1(0〜?), pwm_frequency:12800 Hz, relosution : 10bit 4096 steps
  ledcAttachPin(PWM_L, CHANNEL_L); // set pin:PWM_L to channel 0

  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  ledcSetup(CHANNEL_R, 12800, 10); // channel:2(0〜?), pwm_frequency:12800 Hz, relosution : 10bit 4096 steps
  ledcAttachPin(PWM_R, CHANNEL_R); // set pin:PWM_R to channel 1

  pinMode(CURRENT_SENSE_L, INPUT_PULLUP);
  pinMode(CURRENT_SENSE_R, INPUT_PULLUP);
  pinMode(BATTERY_VOLTAGE_SENSE, INPUT_PULLUP);

  pinMode(DISABLE_L, OUTPUT);
  pinMode(DISABLE_R, OUTPUT);

  // set up interrupt function for motor encoder
  attachInterrupt(ENC_L_A, ISR, CHANGE);
  attachInterrupt(ENC_L_B, ISR, CHANGE);
  attachInterrupt(ENC_R_A, ISR, CHANGE);
  attachInterrupt(ENC_R_B, ISR, CHANGE);
}

void loop() {
  // If Timer has fired
  if (imu_flag == true) {
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);

    get_IMU_data(imu_data);
    complementary_filter(posture_angle, imu_data);

    imu_flag = false;
  }

  //  calculation for PWM output
  int pwm_output_L, pwm_output_R;
  accumulated_angle_error += (posture_angle[1] - target_angle);
  pwm_output_L = -1 * Kp_posture * (posture_angle[1] - target_angle) + Kd_posture * imu_data[3] + Ki_posture * -1 * 0.005 * accumulated_angle_error;

  if (pwm_output_L > 4095) {
    pwm_output_L = 4095;
  }
  else if (pwm_output_L < -4095) {
    pwm_output_L = -4095;
  }

  pwm_output_R = pwm_output_L;


  // check battery voltage before drive motor
  int raw_voltage = analogRead(BATTERY_VOLTAGE_SENSE);
  float voltage = raw_voltage * 3.3 * 4 / 4096;
  bool drive_flag;
  if (voltage > 11) {
    drive_flag = true;
  }
  else {
    drive_flag = false;
  }

  // check robot's posture angle, if it is tilted too much, stop to drive motors.
  if (abs(posture_angle[1]) > (30.0 * M_PI / 180.0)) {
    drive_flag = false;
  }

  if (drive_flag == true) {
    digitalWrite(DISABLE_L, HIGH);
    digitalWrite(DISABLE_R, HIGH);

    if (pwm_output_L >= 0) {
      ledcWrite(CHANNEL_L, pwm_output_L);
      ledcWrite(CHANNEL_R, pwm_output_R);

      digitalWrite(INA_L, HIGH);
      digitalWrite(INB_L, LOW);

      digitalWrite(INA_R, LOW);
      digitalWrite(INB_R, HIGH);
    }

    else {
      digitalWrite(DISABLE_L, HIGH);
      digitalWrite(DISABLE_R, HIGH);
      ledcWrite(CHANNEL_L, abs(pwm_output_L));
      ledcWrite(CHANNEL_R, abs(pwm_output_R));

      digitalWrite(INA_L, LOW);
      digitalWrite(INB_L, HIGH);

      digitalWrite(INA_R, HIGH);
      digitalWrite(INB_R, LOW);
    }
  }
  else {
    digitalWrite(DISABLE_L, LOW);
    digitalWrite(DISABLE_R, LOW);
  }

  current_time = micros();
  passed_time += current_time - last_time;
  last_time = current_time;

  if (COM_FLAG == true) {
    if (Serial.available() >= 3) {
      if (Serial.read() == 'H') {

        byte P_gain_high = Serial.read();
        byte P_gain_low = Serial.read();
        Kp_posture = P_gain_high * 256 + P_gain_low;

        byte I_gain_high = Serial.read();
        byte I_gain_low = Serial.read();
        Ki_posture = I_gain_high * 256 + I_gain_low;

        byte D_gain_high = Serial.read();
        byte D_gain_low = Serial.read();
        Kd_posture = D_gain_high * 256 + D_gain_low;

        velocity_command_linear = float(Serial.read());
        velocity_command_angular = float(Serial.read());

        Serial.println(encoder_count_L);
        Serial.println(encoder_count_R);
        for (int i = 0; i < 3; i++) {
          Serial.println(posture_angle[i]);
        }
        for (int j = 0; j < 10; j++) {
          Serial.println(imu_data[j]);
        }
        Serial.println(voltage);
        //        Serial.println(Kp_posture);
        //        Serial.println(Ki_posture);
        //        Serial.println(Kd_posture);
      }
    }
  }

  else if (COM_FLAG == false) {
    if (passed_time > 10000) {
      //  print at 10Hz (every 100000 usec)
      //    Serial.println(encoder_count_L);
      //    Serial.println(encoder_count_R);
      //    for (int i = 0; i < 3; i++) {
      //      Serial.println(posture_angle[i]);
      //    }
      //    for (int j = 0; j < 10; j++) {
      //      Serial.println(imu_data[j]);
      //    }
      //    Serial.println();
      //    Serial.println(pwm_output_L);
      Serial.println(posture_angle[1]);
      passed_time = 0;
    }
  }
  delay(1);
}
