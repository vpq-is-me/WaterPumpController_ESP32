#Water pump controller.

This controller intended for control submersible pump installed in water well. It turn on pump when pressure in hydraulic accumulator (HA) dropped to lower pressure P1 and switch off when pressure rich high predefined pressure P2. Pressure measured by pressure transducer with range 0-6bar and output 4-20mA. 
For diagnostic reason there is water flow counter with pulse output - 1 pulse per 10litres (dry contact). 

In series with this controller installed external simple pressure switch (optional). It's aim is protection of system in case of controller failure. For normal work this pressure switch must tunned with upper disconnection pressure above controller's upper pressure P2. For exmple if controller's switch off pressure is 3.2 bar external pressure switch can be 3.5 bar. In case of controller failure or for test reason Bypass switch can be operated. It shorted controlle's output switch and in this case external pressure switch mantaine HA pressure.


This controller connected to Bluetooth MESH net and broadcast internal data to [home_automation_server](https://github.com/vpq-is-me/home_auto_nodejs.git) (running on Raspberry Pi).
See project 
Via Bluetooth MESH controller receive state of Sewage Treatment System (STS) - automomous canalization and in case of STS overfill controller stops water feeding.

Also it stops water feeding in case of some failure:
   - dropped air pressure in Hydraulic Accumulator (air leak)
   - pump failure
   - pump capacity dropping 

###Entire system overview

![Alt text](hardware/WPoverview.jpg?raw=true)

###Controller circuit diagram
![Alt text](hardware/circuit.png?raw=true)

