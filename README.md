# Automation of the double drum tack test bench
Automation of a double drum test bench for measuring the tackiness of towpregs, using two control boards, Arduino Mega and Uno, to control the entire workbench and collect data.

## Microcontroller
### Automation Control
Two microcontrollers are used on the double drum tack test bench. They are an **Arduino UNO Rev3** and an **Arduino Mega 2560 Rev3**.

The Arduino UNO is responsible for the measurement of both **peel angle** 𝛽 and **torque** 𝐶, and selected as the master device. 

The Arduino Mega takes on the role of the slave device. It communicates with the master device (Arduino UNO) through an I²C serial communication bus and directly send the measured data in real time without transmitting the signal back to the computer.

### Loadcell Calibration
The script in `LoadCell_Calibration` is used for the calibration of two strain gauge load cells.

## Actuators
**Stepper Motor:** 2 for drums (3A, 24V), 1 for delivery unit (2A, 24V)

**Motor Driver:** DM542 micro-step driver

**Temperature Control Device:** Peltier element and water cooler (with two fans and a pump)

## Sensors
**Angular Position Sensor:** Incremental encoder for the peel angle measurement 

**Force Sensor:** The double drum test bench keeps a constant peel angle during the test and strain gauge load cells measure the value of 𝐶1 and 𝐶2 to calculate the peel force. 

**Temperature Sensor:** The infrared temperature sensor, which is used to detect not only the object temperature but also the ambient temperature.

