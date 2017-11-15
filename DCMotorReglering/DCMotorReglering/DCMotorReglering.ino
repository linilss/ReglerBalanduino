#include "Balanduino.h"
#include "Controller.h"
#include <Arduino.h>		// Standard Arduino header
#include <Wire.h>		// Official Arduino Wire library
#include <Kalman.h>
#include <avrpins.h>

static Kalman kalman;

// Set Kp and Ki here: 
double Kp = 0.1;
double Ki = 4.0; 

double P = 0.0;
double I = 0.0;
double P_I = 0.0;

double reference; // Reference value (In Swedish: Börvärde)

void setup()
{
  initialize();
  Serial.begin(115200);
  
  //
  // Write to serial port for debugging
  //
//  Serial.println();
//  Serial.print("time           ");
//  Serial.print("\t");
//  Serial.print("ref   "); 
//  Serial.print("\t"); 
//  Serial.print("output");
//  Serial.print("\t"); 
//  Serial.print("error "); 
//  Serial.print("\t");   
//  Serial.print("calc u"); // Write the calculated control output
//  Serial.print("\t"); 
//  Serial.println("actual u");  // Write the actual (saturated) control output   
  
  // Setup pulse generator for the setpoint 
  // First argument is the low value of the pulse
  // Second argument is the hight value of the pulse
  // Third argument is the pulse width in microseconds
  setup_setpoint_generator_pulse(6,12,4000000);  
}

void loop()
{
  // Part 1: Sanity check - make sure that the motors are ok
  checkMotors();

  //
  // Part 2: Time management - read actual time and 
  //    calculate the time since last sample time.
  //  
  unsigned long timer_us = micros(); // Time of current sample in microseconds  
  h = (double)(timer_us - pidTimer_us) / 1000000.0; // Time since last sample in seconds
  pidTimer_us = timer_us;  // Save the time of this sampleTime of last sample


  // 
  // Part 3: Generate setpoint value (In Swedish: Börvärde)
  //
  reference = setpoint_generator_pulse();

  //
  // Part 4: Read actual output (In Swedish: Ärvärde)
  //
  double actual_output = getRotSpeed(h);   // Get the speed of the motor in rad/s
  //double actual_output = getPosition();   // Get the position of the motor in rad

  // 
  // Part 5: Generate the control error, i.e. difference
  //    between the reference_value and the actual_output.
  //
  double  e = reference - actual_output; // Control error
  
  //
  // Part 6: Calculate control output (In Swedish: Styrsignal)
  //
  P = Kp * e;
  double u = P_I; // Calculate control output  
  double saturated_u = constrain(u, -12, 12); // Make sure the calculated output is -12 <= u <= 12 (Min voltage -12V, max voltage +12V)
  I = I + Ki * h * e; // Change for PI-controller.
  P_I = P + I;

  
  // 
  // Part 7: Actuate the control output by sending
  //    the control output to the motors
  //
  actuateControlSignal(saturated_u);    
  
  //
  // Write to serial port for debugging
  //
  Serial.print(timer_us/1000000.0, 6);  // Write time (seconds)
  Serial.print("\t");

  Serial.print(reference); // Write reference value
  Serial.print("\t"); 
  
  Serial.print(actual_output); // Write actual output
  Serial.print("\t"); 

  Serial.print(e); // Write control error
  Serial.print("\t"); 
  
  Serial.print(u); // Write the calculated control output
  Serial.print("\t"); 
  
  Serial.println(saturated_u);  // Write the actual (saturated) control output 
}


