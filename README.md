# VL6180
Arduino library to support the VL6180 proximity sensor

## API

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.
The API provides simple distance measure.

## Examples

There are 3 examples with the VL6180 library.

In order to use these examples you need to connect the VL6180 satellite sensor directly to the Nucleo board with wires as explained below:
- pin 1 (INT) of the VL6180 satellite connected to pin A2 of the Nucleo board 
- pin 2 (SCL) of the VL6180 satellite connected to pin D15 (SCL) of the Nucleo board with a Pull-Up resistor of 4.7 KOhm
- pin 3 (GPIO) of the VL6180 satellite connected to pin A1 of the Nucleo board
- pin 4 (SDA) of the VL6180 satellite connected to pin D14 (SDA) of the Nucleo board with a Pull-Up resistor of 4.7 KOhm
- pin 5 (AVDD) of the VL6180 satellite connected to 3V3 pin of the Nucleo board
- pin 6 (GND) of the VL6180 satellite connected to GND of the Nucleo board

* VL6180_Sat_HelloWorld: This example code is to show how to get the proximity
  values of the VL6180 satellite sensor in polling mode.
* VL6180_Sat_Asynchronous: This example code is to show how to get the proximity
  values of the VL6180 satellite sensor using the asynchronous mode.
* VL6180_Sat_Interrupt: This example code is to show how to setup the VL6180
  satellite sensor using the interrupt mode with continuous measurements. At the beginning
  the sensor is configured to generate the interrupt on the new sample ready event;
  pushing the user button, the sensor is configured to generate the interrupt when the
  measurement is lower than 100 mm; pushing the user button again, the sensor is configured
  to generate the interrupt when the measurement is higher than 200 mm; pushing the user
  button again, the sensor is configured to generate the interrupt when the measurement
  is out of the window range between 100 mm and 200 mm; pushing the user button again the
  sensor is configured in the initial state and so on.

## Documentation

You can find the source files at  
https://github.com/stm32duino/VL6180

The VL6180X datasheet is available at  
https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl6180.html
