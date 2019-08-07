#include <TimerOne.h>
#include "MPU9250.h"
#include <Wire.h>
#include <SPI.h>

// use pin IO16 & IO17 for input of the encoder of the left moter, IO34 & IO35 for right motor
#define ENC_L_A 2
#define ENC_L_B 3
#define ENC_R_A 18
#define ENC_R_B 19

// use pin IO3 & IO1 as output of LEFT motor driver INA & INB, pin IO18 & IO5 for RIGHT one
#define INA_L 14
#define INB_L 15
#define INA_R 6
#define INB_R 7

// use pin IO4(A10) as PWM output for LEFT motor driver, pin 2 (A12) as RIGHT one
#define PWM_L 4
#define PWM_R 5

// use pin 36 (A0) as analog read for LEFT motor driver(current sense), pin 39(A3) for RIGHT
#define CURRENT_SENSE_L 54
#define CURRENT_SENSE_R 55

// use pin 56 to display battery voltage
#define BATTERY_VOLTAGE_MONITOR 56

//use pin 25 as disabling LEFT motor driver, pin 26 for RIGHT one.
#define DISABLE_L 30
#define DISABLE_R 8

// configuration of servo city motor : https://www.servocity.com/317-rpm-spur-gear-motor-w-encoder
#define PULSE_PER_ROUND 723.24  //  at output shaft

// parameters to get IMU information
#define GRAVITATIONAL_ACCEL 9.798 //at TOKYO
#define SAMPLING_RATE 200 //IMU referesh rate : 200 Hz (over 1000 Hz is available if you want only refreshing) 
#define SAMPLES_FOR_INITIATION 200 // (not used) get mean value of 200 samples of gyro to cancel native bias
// channel number for timer interruption
#define INTERRUPTION_CHANNEL 0

// use pin 21 & 22 for I2C communication
#define SDA 20
#define SCL 21


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
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

// parameter to trigger imu read process
volatile bool imu_flag = false;
volatile int interrupt_count = 0;

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

long current_time, passed_time, last_time;
int count = 0;



// function that will be called when GPIO interruption was fired
void read_encoder() {
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
void timer_interrupt_function() {
  //  turn on the flag to read IMU
  imu_flag = true;
}


void setup() {
  Serial.begin(115200);
  delay(10);

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

  Timer1.initialize(1000000 / SAMPLING_RATE); //interrupt per 10000 micro seconds(10 msec)
  Timer1.attachInterrupt(timer_interrupt_function);

  // setting pin mode for encoder
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // setting pin mode for motor driver
  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  // set up interrupt function for motor encoder
  attachInterrupt(ENC_L_A, read_encoder, CHANGE);
  attachInterrupt(ENC_L_B, read_encoder, CHANGE);
  attachInterrupt(ENC_R_A, read_encoder, CHANGE);
  attachInterrupt(ENC_R_B, read_encoder, CHANGE);
}


void loop() {
  // If Timer has fired
  if (imu_flag == true) {

    get_IMU_data();
    complementary_filter();
    //    print_time();
    //    Serial.println("");

    imu_flag = false;
    interrupt_count++;
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
    analogWrite(PWM_L, pwm_output_L);
    analogWrite(PWM_R, pwm_output_R);

    digitalWrite(INA_L, HIGH);
    digitalWrite(INB_L, LOW);

    digitalWrite(INA_R, HIGH);
    digitalWrite(INB_R, LOW);
  }

  else {
    analogWrite(PWM_L, abs(pwm_output_L));
    analogWrite(PWM_R, abs(pwm_output_R));

    digitalWrite(INA_L, LOW);
    digitalWrite(INB_L, HIGH);

    digitalWrite(INA_R, LOW);
    digitalWrite(INB_R, HIGH);
  }

  current_time = micros();
  passed_time += (current_time - last_time);
  last_time = current_time;
  //  Serial.println(passed_time);

  if (passed_time > 100000) {
    //  print at 10Hz (every 100000 usec)
    // reserve posture angle for print so that they won't be refreshed during communicating
    roll_print = roll_data;
    pitch_print = pitch_data;
    heading_print = heading_data;


    //    Serial.print(passed_time);
    //    Serial.print(gX);
    //    Serial.print(",");
    //    Serial.print(gY);
    //    Serial.print(",");
    //    Serial.println(gZ);

    Serial.print(roll_print);
    Serial.print(",");
    Serial.print(pitch_print);
    Serial.print(",");
    Serial.println(heading_print);

    //    Serial.print(aX);
    //    Serial.print(",");
    //    Serial.print(aY);
    //    Serial.print(",");
    //    Serial.println(aZ);
  
    //    Serial.println(passed_time);

    //    print_time();
    //  Serial.println();
    passed_time = 0;
    interrupt_count = 0;
  }
}
