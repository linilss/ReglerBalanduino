void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  h = time_since_last_sample(); //samplingstid
  reference = setpoint_generator_pulse(); //börvärde
  double actual_output = getRotSpeed(h); //ärvärde
  double e = reference - actual_output; //reglerfel
  P = c0 * e; // P-del
  double u = P + I; //beräkna styrsignal
  I = I + c1*e; //uppdatera I-delen
  //Begränsa styrsignalen -12 <= u <= 12
  double saturated_u = constrain(u, -12.0, 12.0);
  actuateControlSignal(saturated_u); //aktuera styrsignal

}
