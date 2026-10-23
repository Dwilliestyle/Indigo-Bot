#include <PID_v1.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_enB 11  // PWM
#define L298N_in4 8  // Dir Motor B
#define L298N_in3 7  // Dir Motor B
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 2  // Interrupt (swapped - D2 was faulty)
#define right_encoder_phaseB 4  
#define left_encoder_phaseA 3   // Interrupt (swapped - now on D3)
#define left_encoder_phaseB 5

// Encoders
const double TICKS_PER_REV = 621.0;              // measured 3 turns, R 1874 / L 1850 (was 385)
volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;
volatile char right_wheel_sign = 'p';
volatile char left_wheel_sign = 'p';
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Command timeout
unsigned long last_cmd_millis = 0;
const unsigned long cmd_timeout = 500;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// Feedforward: measured open-loop curve, wheels lifted, average of both wheels.
// FF_SPEED[k] is the speed reached at PWM = 15 + 15*k  (15, 30, 45 ... 255)
const int FF_N = 17;
const double FF_SPEED[FF_N] = {
   0.00,  2.25,  6.38,  9.77, 12.13, 13.75, 14.89, 15.72, 16.31,
  16.78, 17.20, 17.51, 17.80, 17.92, 18.01, 18.31, 18.76
};

// PID (output is now a correction added to the feedforward)
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;
double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;
double Kp_r = 3.0;   // FF: was 11.5
double Ki_r = 7.5;
double Kd_r = 0.0;   // FF: was 0.1
double Kp_l = 3.0;   // FF: was 12.8
double Ki_l = 8.3;
double Kd_l = 0.0;   // FF: was 0.1
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetOutputLimits(-255, 255);  // FF: correction can be negative
  leftMotor.SetOutputLimits(-255, 255);   // FF
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);

  // Flush any garbage on the line before starting
  delay(100);
  while(Serial.available())
  {
    Serial.read();
  }

  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();

    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    }
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    }
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
        last_cmd_millis = millis();
      }
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    noInterrupts();
    unsigned int right_ticks = right_encoder_counter;
    unsigned int left_ticks = left_encoder_counter;
    char right_sign = right_wheel_sign;
    char left_sign = left_wheel_sign;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    right_wheel_meas_vel = (10 * right_ticks * (60.0/TICKS_PER_REV)) * 0.10472;
    left_wheel_meas_vel = (10 * left_ticks * (60.0/TICKS_PER_REV)) * 0.10472;

    // Stop motors if no command received recently
    if(millis() - last_cmd_millis > cmd_timeout)
    {
      right_wheel_cmd_vel = 0.0;
      left_wheel_cmd_vel = 0.0;
    }

    rightMotor.Compute();
    leftMotor.Compute();

    // FF: final PWM = feedforward guess + PID correction
    double right_pwm = feedforward(right_wheel_cmd_vel) + right_wheel_cmd;
    double left_pwm = feedforward(left_wheel_cmd_vel) + left_wheel_cmd;

    // FF: commanded stop -> zero output and clear the PID's integral
    if(right_wheel_cmd_vel == 0.0)
    {
      right_pwm = 0.0;
      resetPID(rightMotor, right_wheel_cmd);
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_pwm = 0.0;
      resetPID(leftMotor, left_wheel_cmd);
    }

    Serial.print('r');
    Serial.print(right_sign);
    Serial.print(right_wheel_meas_vel);
    Serial.print(",l");
    Serial.print(left_sign);
    Serial.print(left_wheel_meas_vel);
    Serial.println(',');
    last_millis = current_millis;

    analogWrite(L298N_enA, (int)constrain(right_pwm, 0, 255));  // FF
    analogWrite(L298N_enB, (int)constrain(left_pwm, 0, 255));   // FF
  }
}

// FF: PWM for a target wheel speed, interpolated from the measured curve
double feedforward(double target)
{
  if(target <= 0.0) return 0.0;
  if(target >= FF_SPEED[FF_N - 1]) return 255.0;
  for(int i = 1; i < FF_N; i++)
  {
    if(target <= FF_SPEED[i])
    {
      double frac = (target - FF_SPEED[i - 1]) / (FF_SPEED[i] - FF_SPEED[i - 1]);
      return 15.0 * i + 15.0 * frac;   // PWM of point i-1 is 15*i
    }
  }
  return 255.0;
}

void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = 'p';
  }
  else
  {
    right_wheel_sign = 'n';
  }
  right_encoder_counter++;
}

void leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = 'n';
  }
  else
  {
    left_wheel_sign = 'p';
  }
  left_encoder_counter++;
}