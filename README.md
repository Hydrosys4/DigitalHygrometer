# DigitalHygrometer
Python Module to interface the Digital Capacitive Hygrometer.

In this folder the python3 library for Interfacing the Capacitive hygrometer. 

BACKGROUND:
The Hygrometer is based on a popular design, which can be found on several chinese supplier with the mane of "Plant watering alarm".
I have modified the firmware to transform it in a sensor, and wrote a dedicated protocol for the communication with microcontroller.
In the folder you can find the python Module for the communication with the hygrometer.
The hygrometer embeds an ATTiny84 which can be reprogrammed using the code in the .c file.

NOTE:
Some of the hygrometers that I have purchased are not functional sinply because the diode in the circuit was inverted ... 
(putting it in the right direction make them fully functional)
Probably the manufacturer was not good enough to properly copy the original design which seems to be coming from this guy here:
https://wemakethings.net/chirp/


