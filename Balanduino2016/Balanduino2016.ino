#include "Balanduino.h"
#include "Controller.h"
#include <Arduino.h>		// Standard Arduino header
#include <Wire.h>		// Official Arduino Wire library
#include <Kalman.h>
#include <avrpins.h>

static Kalman kalman;

// Define the parameters for the inner PID-controller (continuous)

double Kp_inner = 28.53;
double Ki_inner = 241.53;
double Kd_inner = 2.48;
double Tf_inner = 1/25;

double P_inner = 0.0;
double I_inner = 0.0;
double D_inner = 0.0;

double e_inner_old = 0.0;

// Define the parameters for the outer PI-controller (continuous)
double Kp_outer = 0.064 2;
double Ki_outer = -0.033;
double Kd_outer = 0.0;
double Tf_outer = 0.0;

double P_outer = 0.0;
double I_outer = 0.0;
double D_outer = 0.0;

double e_outer_old = 0.0;

double balanduino_pos = 0.0;
double reference; // Reference value (In Swedish: Börvärde)

void setup()
{
  // Initialize all sensor hardwares, filters and observers  
  initialize();
  // Setup pulse generator for the setpoint 
  // First argument is the low value of the pulse
  // Second argument is the hight value of the pulse
  // Third argument is the pulse width in microseconds
  setup_setpoint_generator_pulse(0.0, 1.0, 10000000); 
}

void loop()
{
  //
  // Part 1: Sanity check - make sure that the motors are ok
  //
  checkMotors();

  //
  // Part 2: Time management - read actual time and 
  //    calculate the time since last sample time.
  //  
  unsigned long timer_us = micros(); // Time of current sample in microseconds  
  h = (double)(timer_us - pidTimer_us) / 1000000.0; // Time since last sample in seconds
  pidTimer_us = timer_us;  // Save the time of this sampleTime of last sample

  
  // Part 3: Read the angular orientation (rad) of the robot
  double theta = getTheta();

  // Drive motors if the robot is not lying down. If it is lying down, it has to be put in a vertical position before it starts to balance again.
  // When balancing it has to deviate more ±45 degrees form 0 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
    // The robot is not laying down
  } else {
    // It's no longer laying down
    layingDown = false;

    //
    // Part 4: Read the longitudinal velocity (m/s) of the robot
    //
    double v = getSpeed(h);
    balanduino_pos = balanduino_pos + v * h;
    
    // Outer PI-controller:
    // 
    // Part 5: Generate setpoint value (In Swedish: Börvärde)
    //
    //reference = setpoint_generator_pulse();
    double v_r = 0;

    // 
    // Part 6: Generate the control error for the outer loop, i.e. difference
    //    between the reference_value and the actual_output.
    //
    double e_outer = v_r - v; // Control error
    //double e_outer = reference - balanduino_pos; // Control error
  
    //
    // Part 7: Calculate control output (In Swedish: Styrsignal)
    //
    // Implement your PI-controller here:
    //
    // Pseudo-kod for a PID-controller
    // P = c0 * e
    // D = c1 * D + c2 * (e - eold)
    // u = P + I + D // Bestäm totala styrsignalen
    // daout("u", u) // Skriv styrsignalen till DA-omv.
    // I = I + c3 * e // Uppdatera integraldelen
    // eold = e
    
    P_outer = Kp_outer * e_outer;
    double u_outer = P_outer; // Calculate control output  
    
    // Inner PID-controller:
    // 
    // Part 8: Generate setpoint value for inner controller (In Swedish: Börvärde)
    //
    double theta_r = u_outer;

    // 
    // Part 9: Generate the control error for the inner loop, i.e. difference
    //    between the reference_value and the actual_output.
    //
    double e_inner = theta_r - theta; // Control error

    //
    // Part 10: Calculate control output (In Swedish: Styrsignal)
    //
    // Implement your PID-controller here:
    //
    // Pseudo-kod for a PID-controller
    // P = c0 * e
    // D = c1 * D + c2 * (e - eold)
    // u = P + I + D // Bestäm totala styrsignalen
    // daout("u", u) // Skriv styrsignalen till DA-omv.
    // I = I + c3 * e // Uppdatera integraldelen
    // eold = e
    
    P_inner = Kp_inner * e_inner;
    double u = P_inner; // Calculate control output  
    double saturated_u = constrain(u, -12.0, 12.0); // Make sure the calculated output is -12 <= u <= 12 (Min voltage -12V, max voltage +12V)

    e_inner_old = e_inner;
    e_outer_old = e_outer;

    // 
    // Part 11: Actuate the control output by sending
    //    the control output to the motors
    //
    actuateControlSignal(saturated_u);    
  }


  // Update the motor encoder values
  //updateEncoders();
}

