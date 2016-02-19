# Esc_Cruise_Control
PID control of RC ESC RPM for power/efficiency testing.

Wire a POT for rpm setpoint to speedSettingPin = 0.
Connect esc throttle to escPin = 9.
Connect rpm pulse input to rpmPin = 3.

Motor will hold rpm regardless of Voltage change.

int speedSettingPin = 0;  // analog pin used to connect the potentiometer
const int escPin=9;             // connect to esc
const int rpmPin=3; 

