#include "MPU9250.h"

// use pin IO16 & IO17 for input of the encoder of the left moter, IO34 & IO35 for right motor
#define ENC_L_A 16
#define ENC_L_B 17
#define ENC_R_A 34
#define ENC_R_B 35

// use pin IO3 & IO1 as output of LEFT motor driver INA & INB, pin IO18 & IO5 for RIGHT one
#define INA_L 1
#define INB_L 3
#define INA_R 22
#define INB_R 0

// use pin IO4(A10) as PWM output for LEFT motor driver, pin 2 (A12) as RIGHT one
#define PWM_L A10
#define PWM_R A12
// channels for PWM
#define CHANNEL_L 0
#define CHANNEL_R 1

// use pin 36 (A0) as analog read for LEFT motor driver(current sense), pin 39(A3) for RIGHT
#define ANALOG_IN_L 36
#define ANALOG_IN_R 39

//use pin 25 as disabling LEFT motor driver, pin 26 for RIGHT one.
#define DISABLE_L 25
#define DISABLE_R 26

// configuration of servo city motor : https://www.servocity.com/317-rpm-spur-gear-motor-w-encoder
#define PULSE_PER_ROUND 723.24  //  at output shaft

// parameters to get IMU information
#define GRAVITATIONAL_ACCEL 9.798 //at TOKYO
#define SAMPLING_RATE 200 //IMU referesh rate : 200 Hz (over 1000 Hz is available if you want only refreshing) 
#define SAMPLES_FOR_INITIATION 200 // (not used) get mean value of 200 samples of gyro to cancel native bias
// channel number for timer interruption
#define INTERRUPTION_CHANNEL 0

// use pin 21 & 22 for I2C communication
#define SDA 21
#define SCL 22

// use pin 27 for Chip Select of SPI communication
#define SPI_CHIP_SELECT 27


// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI, SPI_CHIP_SELECT);
int status;


//  parameters for interrupt process
volatile byte pulse_L, last_pulse_L, pulse_R, last_pulse_R;
volatile long count_L = 0, count_R = 0;
//volatile bool LED = false; // for debug

//  parameters for recording time per loop
int time1, time2, dt;
int count_for_output = 0;

//  parameters for motor output shaft state
long current_count_L, last_count_L = 0, current_count_R = 0, last_count_R = 0;
float total_round_L = 0, last_round_L = 0, rpm_L = 0, total_round_R = 0, last_round_R = 0, rpm_R = 0;

//  parameters for PWM output
int pwm_output_L, pwm_output_R;

//  parameters for timer interruption
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// parameter to trigger imu read process
volatile bool imu_flag = false;

// parameter to contain sensor value
float gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, temperature_degC;
// parameter to contain temporal value
float ACCroll, ACCpitch;
// parameter to contain posture angles
float roll_data, pitch_data, heading_data;
float roll_print, pitch_print, heading_print;

// parameter to contain values for initial process : cancel biased gyro value
float sum_gX = 0, sum_gY = 0, sum_gZ = 0, offset_gX = 0, offset_gY = 0, offset_gZ = 0;

// parameters for PID control
int Kp_POSTURE = 2000, Ki_POSTURE = 0, Kd_POSTURE = 50;

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
  count_L = count_encoder(pulse_L, last_pulse_L, count_L);
  count_R = count_encoder(pulse_R, last_pulse_R, count_R);

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
  //  pinMode(21, INPUT_PULLUP);
  //  pinMode(22, INPUT_PULLUP);
  //  delay(10);
  //  Wire.begin(SDA, SCL);
  //  delay(40);


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

    get_IMU_data();
    complementary_filter();
    //    print_time();
    //    Serial.println("");

    imu_flag = false;
  }

  pwm_output_L = -1 * Kp_POSTURE * pitch_data + Kd_POSTURE * gX;
  if (pwm_output_L > 4095) {
    pwm_output_L = 4095;
  }
  else if (pwm_output_L < -4095) {
    pwm_output_L = -4095;
  }

  pwm_output_R = pwm_output_L;

  if (pwm_output_L >= 0) {
    ledcWrite(CHANNEL_L, pwm_output_L);
    ledcWrite(CHANNEL_R, pwm_output_R);

    digitalWrite(INA_L, HIGH);
    digitalWrite(INB_L, LOW);

    digitalWrite(INA_R, HIGH);
    digitalWrite(INB_R, LOW);
  }

  else {
    ledcWrite(CHANNEL_L, abs(pwm_output_L));
    ledcWrite(CHANNEL_R, abs(pwm_output_R));

    digitalWrite(INA_L, LOW);
    digitalWrite(INB_L, HIGH);

    digitalWrite(INA_R, LOW);
    digitalWrite(INB_R, HIGH);
  }

  current_time = micros();
  passed_time += current_time - last_time;
  last_time = current_time;



  if (passed_time > 100000) {
    //  print at 10Hz (every 100000 usec)

    // reserve posture angle for print so that they won't be refreshed during communicating
    roll_print = roll_data;
    pitch_print = pitch_data;
    heading_print = heading_data;


    //    Serial.print(gX);
    //    Serial.print(",");
    //    Serial.print(gY);
    //    Serial.print(",");
    //    Serial.println(gZ);

    Serial.print(roll_data);
    Serial.print(",");
    Serial.print(pitch_data);
    Serial.print(",");
    Serial.println(heading_data);

    //    Serial.println(passed_time);
    //  Serial.println();
    passed_time = 0;
  }
}
