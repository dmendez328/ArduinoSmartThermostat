Project Overview:

This project implements a microcontroller-based temperature monitoring and control system using an AVR microcontroller. The system utilizes a DS18B20 temperature sensor, an LCD display, buttons, a rotary encoder, LEDs, and a buzzer. Key functionalities include temperature monitoring, range-based notifications, and user-adjustable high/low temperature thresholds.


Features:

Temperature Monitoring: Reads temperature data in real-time from a DS18B20 sensor and displays it in Fahrenheit on an LCD.
Adjustable Thresholds: Uses a rotary encoder to adjust high and low temperature limits, displayed on the LCD.


LED Indicators:

Green: Temperature is within range.
Yellow: Temperature is below the low threshold (heater should activate).
Red: Temperature is above the high threshold (cooler should activate).
Audible Alerts: A buzzer plays a note when the temperature deviates significantly (±3°F) from the thresholds.
Servo Motor Integration: Adjusts based on temperature or user interaction, demonstrating dynamic control.
Persistent Settings: High and low threshold values are saved to EEPROM, ensuring retention across power cycles.


User Interface: Includes buttons and an LCD splash screen for improved usability.


Hardware Components:

AVR Microcontroller: Core of the system, responsible for processing and control.
DS18B20 Sensor: Measures temperature in real-time.
LCD Display: Outputs temperature readings, threshold values, and user interface details.
LEDs: Indicate the temperature status relative to thresholds.
Rotary Encoder: Adjusts temperature thresholds interactively.
Buttons: Allow manual input for additional user controls.
Buzzer: Provides audible feedback for out-of-range temperature conditions.
Servo Motor: Demonstrates dynamic mechanical response to temperature changes.


Software Features:

Interrupt-Driven Design: Pin change interrupts for rotary encoder and buttons enhance responsiveness.
Timers: Three timers configured for precise control of servo motor, buzzer, and other periodic operations.
EEPROM Storage: Retains user-defined high/low temperature thresholds across resets.
State Machines: Efficiently handles button and rotary encoder inputs for seamless user interaction.
Frequency Control: Dynamically generates tones for the buzzer based on preset note frequencies.
Key Functionalities
Temperature Reading and Conversion: Processes raw sensor data into Fahrenheit values with one decimal precision.
User-Adjustable Limits: Rotary encoder adjusts high and low temperature thresholds displayed in real-time.
Dynamic Servo Motor Control: Motor position changes based on current temperature or selected threshold.
LED Indication Logic: Color-coded LEDs signal temperature status, guiding user response.
Buzzer Alerts: Notifies when temperature exceeds the specified deviation range from thresholds.


How to Run:

Connect the hardware components as per the schematic.
Flash the provided code to the AVR microcontroller using an appropriate programmer.
Power up the system. The LCD will display a splash screen before entering the main operational state.
Adjust the high and low temperature limits using the rotary encoder.
Observe temperature readings, LED indicators, and audible alerts based on the current temperature status.


File Structure:

main.c: Contains the core program logic, including temperature monitoring, user input handling, and hardware control.
lcd.c & lcd.h: Manage LCD initialization and display operations.
ds18b20.c & ds18b20.h: Driver for the DS18B20 temperature sensor.
EEPROM Functions: Handle saving and loading threshold values.
Timer Functions: Configure and manage the timers used for motor control and buzzer output.
