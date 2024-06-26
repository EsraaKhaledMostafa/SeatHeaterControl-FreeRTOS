# SeatHeaterControl-FreeRTOS

## Used Hardware:
- Tiva-C Microcontroller.

## SW Drivers:
- MCAL:
GPIO
GPTM
ADC
UART

- HAL:
LM-35 Temperature Sensor

- APP

## Pdf for Project Results: 
[Results](https://github.com/EsraaKhaledMostafa/SeatHeaterControl-FreeRTOS/blob/main/RTOS_FinalProject_SubmissionDoc.pdf)


## Specifications:
1. For each seat the user shall input the required level of heating (off, low, medium, or high) where
initially off is set and each press goes one step further (from off to low, from low to medium, from
medium to high, and from high to off once again).

2. The heater will be driven from a signal to control its intensity:
- If the current temperature is less than the desired temperature by 10°C or more the heater
should be enabled with the high intensity.
- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
should be enabled with a medium intensity.
- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
be enabled with a low intensity.
- If the current temperature is more than the desired temperature the heater should be
disabled.
- The heater shall be enabled once again if the temperature becomes less than the desired
temperature by 3°C.
- Note that for the purpose of testing only the heater level shall control the LED:
    i. Green color indicates low intensity.
    ii. Blue color indicates medium intensity.
    iii. Cyan color indicates high intensity.

3. The temperature sensor shall be connected to the ADC so that the current temperature is measured
correctly. Note that only the range 5°C-40°C shall be treated as the valid range.

4. The current temperature, the heating level, and the heater state should be displayed on the screen by
sending it through the UART.

5. If failure was detected in the temperature sensor the seat assigned to such sensor shall stop from
controlling the temperature and the red LED shall be ON to inform the user that there is a problem
with the sensor.
