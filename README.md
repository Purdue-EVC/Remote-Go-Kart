# Rc Go Kart
The Purdue EVC Club's code for the Rc Go kart.
## Setup
Currently using a red line 775 for the steering motor with a SparkMini as the motor controller.
Then using a Rev Robotics Through Bore Encoder for the steering shaft's positioning.
For the brakes we use a linear actuator with a potentiometer as the position sensor.
For motor control we use a sevcon gen4 size 4 with an unknown decade old brushless dc motor.
There is an stm32 controlling the entire kart.
For the remote controller we use a FlySky ia10b receiver with a FlySky i6X for the transmitter.
## Power
Power for all devices except the main motor is entirely from battery power passed through a 48v-12v/5v dc to dc converter. 
12v is supplied to the front steering, braking and the stm32. 5v is supplied to all other devices on the kart.
For the sevcon we use a keyed ignition start along with 2 estops in series connected to the contactor for the sevcon.
Battery power is supplied directly to the sevcon through the contactor.
