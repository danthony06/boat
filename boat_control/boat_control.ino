#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>

#define DEBUG_PRINT (true)

#define RUDDER_PIN (5)
#define THROTTLE_PIN (6)

#define RUDDER_MAX (1900)
#define THROTTLE_MAX (1900)

#define RUDDER_MIN (1100)
#define THROTTLE_MIN (1100)

#define RUDDER_RANGE (RUDDER_MAX - RUDDER_MIN)
#define THROTTLE_RANGE (THROTTLE_MAX - THROTTLE_MIN)

#define RUDDER_MID (((RUDDER_MAX - RUDDER_MIN) / 2) + RUDDER_MIN)
#define THROTTLE_MID (((THROTTLE_MAX - THROTTLE_MIN) / 2) + THROTTLE_MIN)


int rudder_raw;
int throttle_raw;
int rudder_offset;
int throttle_offset;
int rudder_scaled;
int throttle_scaled;
double response_mag;
int motor_left;
int motor_right;

int left_mix;
int right_mix;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor* left_motor = AFMS.getMotor(1);
Adafruit_DCMotor* right_motor = AFMS.getMotor(2);

void setup()
{
  pinMode(RUDDER_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);

  AFMS.begin();
  left_motor->setSpeed(0);
  right_motor->setSpeed(0);
  left_motor->run(RELEASE);
  right_motor->run(RELEASE);

#if DEBUG_PRINT
  Serial.begin(9600);
#endif
}

void loop()
{
  // Read the pulse widths of the inputs
  rudder_raw = pulseIn(RUDDER_PIN, HIGH);
  throttle_raw = pulseIn(THROTTLE_PIN, HIGH);

  // Occasionally we get glitches when we are at the limits of the physical control
  rudder_raw = constrain(rudder_raw, RUDDER_MIN, RUDDER_MAX);
  throttle_raw = constrain(throttle_raw, THROTTLE_MIN, THROTTLE_MAX);

  // Map the raw PWM values to the range [-255, 255]
  rudder_scaled = map(rudder_raw, RUDDER_MIN, RUDDER_MAX, 255, -255);
  throttle_scaled = map(throttle_raw, THROTTLE_MIN, THROTTLE_MAX, -255, 255);

  // Evenly mix the throttle and rudder response
  left_mix = rudder_scaled + throttle_scaled;
  right_mix = -rudder_scaled + throttle_scaled;

  // If the combined response is greater than the maximum allowable, rescale to the
  // maximum value. Scale both throttle and rudder so they are proportionally reduced
  if ((left_mix > 255) ||
    (right_mix > 255) ||
    (left_mix < -255) ||
    (right_mix < -255))
  {
    response_mag = 255.0 / (double)((rudder_scaled * rudder_scaled) + (throttle_scaled * throttle_scaled));
    left_mix = (int)((double)left_mix * response_mag);
    right_mix = (int)((double)right_mix * response_mag);
  }

  // Set the new speed
  left_motor->setSpeed(abs(left_mix));
  right_motor->setSpeed(abs(right_mix));

  // Set the motor directions
  if (left_mix >= 0)
  {
    left_motor->run(FORWARD);
  }
  else
  {
    left_motor->run(BACKWARD);
  }

  if (right_mix >= 0)
  {
    right_motor->run(FORWARD);
  }
  else
  {
    right_motor->run(BACKWARD);
  }

  // Print the values we recorded
#if DEBUG_PRINT
//  Serial.print("Rudder Raw: ");
//  Serial.println(rudder_raw);

//  Serial.print("Throttle Raw: ");
//  Serial.println(throttle_raw);

//  Serial.print("Rudder Offset: ");
//  Serial.println(rudder_offset);

//  Serial.print("Throttle Offset: ");
//  Serial.println(throttle_offset);

//  Serial.print("Rudder Scaled: ");
//  Serial.println(rudder_scaled);

//  Serial.print("Throttle Scaled: ");
//  Serial.println(throttle_scaled);

  Serial.print("Left mix: ");
  Serial.println(left_mix);

  Serial.print("Right mix: ");
  Serial.println(right_mix);

//  delay(10000);
#endif
}
