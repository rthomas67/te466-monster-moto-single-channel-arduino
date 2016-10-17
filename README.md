# te466-monster-moto-single-channel-arduino
This is Arduino sketch code to drive the TE466 (SparkFun knock-off) single channel motor shield.

# TE466
This single channel motor controller is essentially one half of the circuitry that
is on the SparkFun Monster Moto Shield.  It has connections to control only one
DC motor using PWM to control the speed and logic-level (5V) inputs to determine
which direction the motor runs.

# Arduino Code Notes
The code is designed to use several classes that represent a basic state machine
for the motor that is checked and adjusted every time the program loops. This
allows the motor current and input switch status to be checked very frequently
instead of holding up the loop with "delay()" call.  It also simplifies the 
definition of a sequence of timed "ProgramStep" objects to control what the motor
does.

# Purpose
The reason for choosing a motor shield that can handle higher amperage, and
the motivation for writing the Arduino code in a way that makes it simple
and flexible to define a pre-set sequence of motor actions, is that this
was intended to go in an "ammo-box/wiper-motor jigging machine" for ice
fishing.

The motor programs can be set to keep a lure in constant motion, twitch
it every few seconds, move it up and down very slowly, lift it off the
bottom every few minutes, or whatever pattern of jigging motion seems
like it might attract the attention of a lethargic, cold, winter
fish.


