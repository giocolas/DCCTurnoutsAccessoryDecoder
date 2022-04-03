# DCC Turnouts Accessory Decoder based on Arduino, Adafruit PCA9685 and SG90 servo-motors
![License](https://img.shields.io/badge/License-MIT-green)

*Versione in italiano [qui](README.md)*

## Table of contents
- [Introduction](#introduction)
- [What is needed](#what-is-needed)
- [Connections](#connections)
- [Configuration](#configuration)
- [Table of CVs (configuration variables) used](#table-of-cvs-configuration-variables-used)
- [Some specifics on how the sketch works](#some-specifics-on-how-the-sketch-works)
- [Version and Contributors](#version-and-contributors)

## Introduction
This sketch for [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) implements a complete accessory-type DCC decoder to drive servo motors of the [SG90](https://google.com/search?q=servo+sg90) type to move the switch needles of your model railway

![Overview](/images/DCC_Turnouts_Overview.jpg)

## What is needed
To create a complete servo drive DCC accessory-decoder, you need:
* a board [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) or [compatible](https://google.com/search?q=geekcreit+arduino+uno)
* an interface [Adafruit PCA9685](https://adafruit.com/product/815) (demo available [here](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all)) capable of simultaneously operating 16 servos; PCA9685 uses the Arduino I2C communication bus (therefore with only 2 pins used) and it's possible to insert multiple interfaces in cascade [bridging](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all#chaining-drivers) some pins dedicated to the purpose
* an external 5V DC power supply, required by the PCA9685 interface, to drive the servos
* a shield or interface that is able to "read" the DCC signal through Arduino, I highly recommend [shield or interface](https://github.com/lucadentella/arduino-dccshield) of my friend Luca Dentella

## Connections
* Arduino Uno or compatible interface
    - use the USB cable (useful if you want to monitor the traffic on the serial monitor window) or the appropriate 5V power plug
* DCC interface / shield
    - use the appropriate DCC terminal block of the interface / shield to which you will connect the signal coming from your control unit, the DCC signal works in AC mode and therefore you can connect both one way and the other
    - VCC must be connected to one of the dedicated 5V Arduino pins (voltage)
    - GND must be connected to one of the dedicated GND Arduino pins (ground)
    - DCC must be connected to the pin dedicated to the data input with interrupt, pin declared for this sketch is 2
    - ACK must be connected to the pin dedicated to the ACK signal, the pin declared for this sketch is 6

![Overview](/images/DCC_Turnouts_DCCInputToArduinoInterface.jpg)

* PCA9685
    - VCC must be connected to one of the dedicated 5V Arduino pins (voltage)
    - GND must be connected to one of the dedicated GND Arduino pins (ground)
    - SDA must be connected to pin A4
    - SCL must be connected to pin A5
    - use the PWM pins already colored to facilitate the connection with your servo motors (yellow => PWM, red => VCC, black => GND)
> ** An external 5V power supply is required to drive the servo motors, this approach avoids the possible current overload on the Arduino!**

![Overview](/images/DCC_Turnouts_PCA9685ConnectionDetail.jpg)

## Configuration

The sketch includes this configuration:
- definition of the number of servos managed, the default value is 2
- definition of the movement interval between one cycle and another, the default value is 70 (in milliseconds) in order to achieve a slow and realistic movement of the needles
- definition of the angle / initial position and angle / final position by means of a specific configuration
- definition of whether the movement is to be carried out from right to left or from left to right (reverse way)

## Table of CVs (configuration variables) used

| CV | Description | Notes | Default value |
|:--|:--|:--|--:|
| 33 | Interval between a single position movement and the next | ms | 70 |
| 34 | Minimum servo position (high bits) | | 0 |
| 35 | Minimum servo position (low bits) | | 0 |
| 36 | Maximum servo position (high bits) | | 0 |
| 37 | Maximum servo position (low bits) | | 50 |
| 38 - 53 | Reverse mode (38 = servo #1, 39 = servo #2, etc ...) - 0 = normal, any non-zero value = inverted | | 0 |
| 54 - 69 | Servo position (54 = servo #1, 55 = servo #2, etc.) - 0 = closed, any value other than zero = open | | 0 |

> Value range allowed for positioning the servo goes from 0 to 4096 and it was necessary to use 2 distinct CVs
> as shown in the table above, both the minimum and maximum positions are indicated with high bits and low bits
> the formula used to define the exact position is ** position = (high_bits * 256) + low_bits **

> Difference between minimum and maximum position is by default set to 50
> as very little movement is required to move the switch needles

## Some specifics on how the sketch works

***
Structure containing the status of each servo:
- **moving** indicates whether the servo is moving or not
- **positionState** indicates the set position
- **actualPosition** indicates the current position on the basis of which it is decided whether to move towards the minimum value or towards the maximum value, at each loop cycle of the sketch it is increased or decreased by one unit until reaching the desired position; the loop cycle evaluates the state and movement of all servos, thus allowing simultaneous actions to be performed on all connected servos
```c
typedef struct ServoStatus {
  bool moving;
  ServoPositionState positionState;
  long actualPosition;
};
```
***
In the initial setup, when the Arduino is turned on, a cycle is performed for each servo where the inverse mode is: initialized, the current position read and the relative position set
```c
void setup() {
    ...
    for (int idx = 0; idx < NUM_SERVOS; idx++) {

        // Get servo states
        ServoPositionState positionState = (Dcc.getCV(OFFSET_POSITION_CV_VALUE + idx) != 0 ? THROWN : CLOSED);
        ServoReversedMode reversedMode = (Dcc.getCV(OFFSET_REVERSED_CV_VALUE + idx) != 0 ? ENABLED : DISABLED);

        // Init servo position & state
        servoStatus[idx].moving = false;
        servoStatus[idx].positionState = positionState;
        servoStatus[idx].actualPosition = getServoPosition(positionState, reversedMode);
        pwm.setPWM(idx, 0, servoStatus[idx].actualPosition);
    
    }
    ...
}
```
***
Each loop cycle of the sketch it's checked whether there is a DCC command to be processed through the appropriate callback functions provided by the library. This approach allows incoming commands to be processed to allow execution of the opposite movement even when the previous instruction is still executing
```c
void loop() {
    ...
    // Process DCC incoming packets
    Dcc.process();
    ...
}
```
***
Cycle execution to simultaneously perform the movement on each individual servo until the desired position is reached
```c
void loop() {
    ...
        // Loop for each servo
        for (int idx = 0; idx < NUM_SERVOS; idx++) {

            // Execute servo movement
            if (servoStatus[idx].moving) {

                // Get reversed mode & actual position
                ServoReversedMode reversedMode = (Dcc.getCV(OFFSET_REVERSED_CV_VALUE + idx) != 0 ? ENABLED : DISABLED);
                long actualPosition = servoStatus[idx].actualPosition;

                // Apply min or max based on position, reversed mode and movement type to be made
                if ((servoStatus[idx].positionState == THROWN && reversedMode == DISABLED) ||
                    (servoStatus[idx].positionState == CLOSED && reversedMode == ENABLED)) {
                    executeSingleMovementToMinValue(idx);
                } else {
                    executeSingleMovementToMaxValue(idx);
                }

                // Set moved to exec delay
                moved = true;
            }
        }

        // Apply delay between movement and next
        if (moved) {
            delay(servoDelay);
        }
    ...
}
```
***
Function that performs the movement of the servo towards the minimum position by decreasing the value of the current position by 1 unit and verifying that the movement has reached the desired limit value
> The **executeSingleMovementToMaxValue** function is identical to this one but executes the movement towards the maximum position
```c
void executeSingleMovementToMinValue(int idx) {
    // Get actual position
    long actualPosition = servoStatus[idx].actualPosition; 

    // Check if position is over min value
    if (actualPosition < servoMinValue) {
        actualPosition = servoMinValue;
    }

    // Check if there is an update to new servo position
    if (actualPosition > servoMinValue) {
        actualPosition--;
    }

    // Set new servo position
    pwm.setPWM(idx, 0, actualPosition);

    // Update last status values
    servoStatus[idx].actualPosition = actualPosition;
    servoStatus[idx].moving = (actualPosition != servoMinValue);
}
```
***
Callback function of the change position of an exchange, it is verified if the command is directed to one of the servos managed by the sketch Checking the main address assigned to the decoder according to the number of managed servos
> **Example:**
> Address assigned to the decoder: 10
> Number of managed servo motors: 4
> Valid addresses: 10 => first servo motor, 11 => second servo motor ... 13 => fourth servo motor
```c
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {
    // Get decoder DCC address
    long dccAddress = Dcc.getAddr();

    // Check if DCC packets needs to be processed on one of the servos
    if (Addr >= dccAddress && Addr < dccAddress + NUM_SERVOS) {
  
        // Calculate no of servo
        int servoNum = Addr - dccAddress;
    ...
}
```
***

## Version and Contributors
Version: 1.0.0
Published date: April 2022
Author: [Giovanni Colasurdo](mailto:gio.colasurdo@gmail.com)