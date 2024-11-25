# I2C LCD Clock
Displays the time in the top center of the LCD. The bottom left and right will contain references for button inputs. One button will display a menu and the other will select the menu input. A potentiometer will be used to read values to set the clock time.

# Hardware
- 16x02 LCD Screen
- STM32-F446RE
- 2 Buttons
- Potentiometer
- Jumper Wires

# Features
- I2C Communication
- Timer interrupt to advance clock

# Will add in future (maybe)... Want to work on more projects
- User can set the clock via button selects
- Menu driven architecture to select modes (12hr/24hr, AM/PM)
- Potentiometer values converted from analog to digital via ADC 

# Pins 
- 5V to breadboard + 
- GND to breadboard -
- PB8 to breadboard to SCL
- PB9 to breadboard to SDA
- PB4 to breadboard to resistor to button + 5V/GND 
- PB5 to breadboard to resistor to button + 5V/GND
- PA0 to breadboard to potentiometer + 5V/GND


![LCD_Screen](/images/LCD_Screen.jpeg)