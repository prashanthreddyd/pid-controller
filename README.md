# pid-controller
This repo contains a header and cpp code of a PID-controller

1. This PID controller implements a closed loop variable control.Variable can be any 
   parameter in a control system and calculates th output that affects the variable.
   Example: Calculating a PWM value to control speed of a brushed DC motor speed contol
2. Saturation limits and any other conditions should be implemented as specific to an
   application. Because this PID controller implements generic PID algorithm.
