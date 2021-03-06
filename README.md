# Automated Fan #
- - - -
An automated temperature controlled fan using the LPC802 microcontroller.

*Items used
  * LPC802 Development Board
  * LM74 Temperature Sensor
  * 16 Channel ADC-MUX
  * Corsair 12V DC Fan
  * 2-channel 5v Relay
  * Seven Segment display
  
- - - -
## System Usage ## 
The system will provide the user with the option to switch between automated mode and
manual mode. The system modes can be set through the mounted button beside the fan as shown in figure below. The four seven-segment LEDs that are on the left side would display the ambient room temperature when the system is turned on and when the system is turned off it would display ‘-off’. The single seven-segment display on the right shows the fan speed when set in manual mode. The fan
speeds vary from levels 1 to 3, one being the slowest and 3 being the fastest. The figure below shows the displays
when the system is turned on and on manual mode with fan speed set to 3.

 <img src="https://github.com/wmaneesh/AutomatedFan/blob/master/images/final_product.jpg" width="200">

- - - -
## Acrchitecture ## 

* Software Artchitectures Used
  * SysTick Timer (System Timer for reading tempearture sensor periodically)
  * I2C Serial Communication Protocol (Communicating with temperature sensor)
  * Switch Matrix (PWM signal for fan)
  
 <img src="https://github.com/wmaneesh/AutomatedFan/blob/master/images/Software_Architecture.png" width="600">

