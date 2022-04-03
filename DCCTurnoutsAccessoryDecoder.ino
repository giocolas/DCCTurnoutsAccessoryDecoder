/*
 * DCC Turnouts accessory decoder based on:
 *      - Arduino Uno or other Arduino compatible board
 *      - DCC Shield by Luca Dentella (see https://github.com/lucadentella/arduino-dccshield)
 *      - Adafruit PCA9685 or other compatible interfaces (see https://www.adafruit.com/product/815)
 * 
 * Release: 1.0.0
 * 
 * Author: Giovanni Colasurdo
 * Date: April 2022
 * 
 */

/*
 * Please comment this line if you don't want to debug this sketch on the serial console
 */
#define DEBUG 1

/*
 * Include referenced libraries
 */
#include <NmraDcc.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*
 * NMRA DCC library
 */
NmraDcc Dcc;

/*
 * Adafruit PCA9685 PWM Servo driver, default address is 0x40
 */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*
 * Trasform 2 integer bytes (high & low part) into long value
 */
#define LONG_VALUE(HIGH_BITS, LOW_BITS) ((HIGH_BITS * 256) + LOW_BITS)

/*
 * Board pin used to get DCC packets by DCC shield
 * Board pin used to send DCC ACK signal
 */
#define DCC_PIN 2
#define ACK_PIN 6

/*
 * PWM servo frequency
 */
#define PWM_FREQ 60

/*
 * Number of servos: first is servo no.0, second is servo no.1, etc..
 */
#define NUM_SERVOS 2

/*
 * Servo CVs table:
 *      - 33      => delay in milliseconds between single step movement and next, DEFAULT => 10
 *      - 34      => min servo position value (high bits)
 *      - 35      => min servo position value (low bits), DEFAULT => 250
 *      - 36      => max servo position value (high bits)
 *      - 37      => max servo position value (low bits), DEFAULT => 400
 *      - 38 - 53 => non-zero value is intended in reversed mode actions
 *                   e.g. depending on how servo is mounted, normally 0 -> closed & 1 -> thrown; reversed mode 0 -> thrown & 1 -> closed
 *                   each address / servo: 38 -> servo no.0, 39 -> servo no.1, ... 53 -> servo no.15
 *      - 54 - 69 => turnout position, 0 -> closed & 1 -> thrown
 *                   each address / servo: 54 -> servo no.0, 55 -> servo no.1, ... 69 -> servo no.15
 */
#define DELAY_CV_VALUE            33

#define DELAY_DEFAULT_VALUE       70

#define MIN_HIGH_CV_VALUE         34
#define MIN_LOW_CV_VALUE          35
#define MAX_HIGH_CV_VALUE         36
#define MAX_LOW_CV_VALUE          37

#define MIN_HIGH_DEFAULT_VALUE    0
#define MIN_LOW_DEFAULT_VALUE     0   // default value = (0 * 256) + 0  => 0
#define MAX_HIGH_DEFAULT_VALUE    0
#define MAX_LOW_DEFAULT_VALUE     50  // default value = (0 * 256) + 50 => 50

#define OFFSET_REVERSED_CV_VALUE  38

#define OFFSET_POSITION_CV_VALUE  54

/*
 * Declare enums
 */
enum ServoReversedMode {
    DISABLED,
    ENABLED
};

enum ServoPositionState {
    CLOSED,
    THROWN
};

/*
 * Servo delay, position values & status structure
 */
int servoDelay;
long servoMinValue;
long servoMaxValue;

typedef struct ServoStatus {
  bool moving;
  ServoPositionState positionState;
  long actualPosition;
};

ServoStatus servoStatus[NUM_SERVOS];

/*
 * Special sketch states (used to optimize factory reset servo movements)
 */
bool resettingState = false;

/*
 * Initial setup of board
 */
void setup() {

    // Set serial baud rate
#ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Starting ...");
#endif

    // Init NmraDcc library & set ACK pin to output
    Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, false);
    Dcc.init(MAN_ID_DIY, 1, FLAGS_DCC_ACCESSORY_DECODER | FLAGS_OUTPUT_ADDRESS_MODE, 0);
    pinMode(ACK_PIN, OUTPUT);
#ifdef DEBUG
    Serial.print("\tNmraDcc library initialized, DCC Pin: "); Serial.print(DCC_PIN); Serial.print(", ACK Pin: "); Serial.print(ACK_PIN); Serial.println();
#endif
  
    // Init Adafruit PCA9685 interface
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);
    delay(100);
#ifdef DEBUG
    Serial.print("\tPWM library correctly initialized, Frequency: "); Serial.print(PWM_FREQ); Serial.println();
#endif

    // Set initial values of delay and min & max servos position value
    setDelay();
    setServoValues();

#ifdef DEBUG
    Serial.print("\tDelay value :"); Serial.print(servoDelay); Serial.println();
    Serial.print("\tDCC address :"); Serial.print(Dcc.getAddr()); Serial.println();
    Serial.print("\tMin position value :"); Serial.print(servoMinValue); Serial.println();
    Serial.print("\tMax position value :"); Serial.print(servoMaxValue); Serial.println();
#endif

    // For each servo init & get previous status
    for (int idx = 0; idx < NUM_SERVOS; idx++) {

        // Get servo states
        ServoPositionState positionState = (Dcc.getCV(OFFSET_POSITION_CV_VALUE + idx) != 0 ? THROWN : CLOSED);
        ServoReversedMode reversedMode = (Dcc.getCV(OFFSET_REVERSED_CV_VALUE + idx) != 0 ? ENABLED : DISABLED);

        // Init servo position & state
        servoStatus[idx].moving = false;
        servoStatus[idx].positionState = positionState;
        servoStatus[idx].actualPosition = getServoPosition(positionState, reversedMode);
        pwm.setPWM(idx, 0, servoStatus[idx].actualPosition);
    
#ifdef DEBUG
        Serial.print("\tServo no. "); Serial.print(idx);
        Serial.print(", state "); Serial.print(positionState == THROWN ? "Thrown" : "Closed");
        Serial.print(", reversed "); Serial.print(reversedMode == ENABLED ? "True" : "False");
        Serial.println();
#endif
    }
}

/*
 * Board main loop
 */
void loop() {

    // Process DCC incoming packets
    Dcc.process();

    // Check is there are movements
    if (isMoving()) {

        // It's used to know whether to apply delay between one movement and another
        bool moved = false;
            
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
    }
}

/*
 * Get initial position to min or max value based on last servo status
 */
long getServoPosition(ServoPositionState positionState, ServoReversedMode reversedMode) {
    return (positionState == THROWN
           ? (reversedMode == ENABLED ? servoMaxValue : servoMinValue)
           : (reversedMode == ENABLED ? servoMinValue : servoMaxValue));
}

/*
 * Check if there are any movements of the servos to be made
 */
bool isMoving() {
    for (int idx = 0; idx < NUM_SERVOS; idx++) {
        if (servoStatus[idx].moving) {
            return true;
        }
    }

    return false;
}

/*
 * Execute servo-n single movement to min value position
 */
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

/*
 * Execute servo-n single movement to max value position
 */
void executeSingleMovementToMaxValue(int idx) {
    // Get actual position
    long actualPosition = servoStatus[idx].actualPosition; 

    // Check if position is over max value
    if (actualPosition > servoMaxValue) {
        actualPosition = servoMaxValue;
    }

    // Check if there is an update to new servo position
    if (actualPosition < servoMaxValue) {
        actualPosition++;
    }

    // Set new servo position
    pwm.setPWM(idx, 0, actualPosition);

    // Update last status values
    servoStatus[idx].actualPosition = actualPosition;
    servoStatus[idx].moving = (actualPosition != servoMaxValue);
}

/*
 * Init or set delay
 */
void setDelay() {
    servoDelay = Dcc.getCV(DELAY_CV_VALUE);

    // Check if there is init value
    if (servoDelay == -1) {
        servoDelay = DELAY_DEFAULT_VALUE;
    }
}

/*
 * Init or set servo position values (min & max)
 */
void setServoValues() {
    int high, low;

    // Get high & low bits of min value
    high = Dcc.getCV(MIN_HIGH_CV_VALUE);
    low  = Dcc.getCV(MIN_LOW_CV_VALUE);

    // Check if there are init values
    if (high == -1 && low == -1) {
        high = MIN_HIGH_DEFAULT_VALUE;
        low  = MIN_LOW_DEFAULT_VALUE;
    }

    // Set min value
    servoMinValue = LONG_VALUE(high, low);

    // Get high & low bits of max value
    high = Dcc.getCV(MAX_HIGH_CV_VALUE);
    low  = Dcc.getCV(MAX_LOW_CV_VALUE);

    // Check if there are init values
    if (high == -1 && low == -1) {
        high = MAX_HIGH_DEFAULT_VALUE;
        low  = MAX_LOW_DEFAULT_VALUE;
    }

    // Set max value
    servoMaxValue = LONG_VALUE(high, low);
}

/*
 * Set immediately servo position (used when resetting factory values, change reversed mode or set new limit position values
 */
void resetServoPosition(bool allServos, int servoNum) {

    // Set min & max index to reset single servo or all servos
    int minIndex = (allServos ? 0 : servoNum);
    int maxIndex = (allServos ? NUM_SERVOS : servoNum + 1);

    for (int idx = minIndex; idx < maxIndex; idx++) {

        // Resetting single servo movement
        servoStatus[idx].moving = false;

        // Get reversed mode & actual position
        ServoReversedMode reversedMode = (Dcc.getCV(OFFSET_REVERSED_CV_VALUE + idx) != 0 ? ENABLED : DISABLED);
        long actualPosition = servoStatus[idx].actualPosition;

        // Apply min or max based on position, reversed mode and movement reset to be made
        if ((servoStatus[idx].positionState == THROWN && reversedMode == DISABLED) ||
            (servoStatus[idx].positionState == CLOSED && reversedMode == ENABLED)) {
            if (actualPosition != servoMinValue) {
                actualPosition = servoMinValue;
            }
        } else {
            if (actualPosition != servoMaxValue) {
                actualPosition = servoMaxValue;
            }
        }
        servoStatus[idx].actualPosition = actualPosition;

        // Reset servo position
        pwm.setPWM(idx, 0, actualPosition);
    }
}

/*
 * Callback function used to get turnout commands
 */
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {

    // Get decoder DCC address
    long dccAddress = Dcc.getAddr();

#ifdef DEBUG
    Serial.print("DCC command received, address: " ); Serial.print(Addr, DEC); Serial.print(", direction: "); Serial.println(Direction, DEC);
#endif

    // Check if DCC packets needs to be processed on one of the servos
    if (Addr >= dccAddress && Addr < dccAddress + NUM_SERVOS) {
  
        // Calculate no of servo
        int servoNum = Addr - dccAddress;

        // Set servo state to thrown or closed based on "Direction" parameter
        if (Direction == 0) {
            if (servoStatus[servoNum].positionState != CLOSED) {
                servoStatus[servoNum].moving = true;
                servoStatus[servoNum].positionState = CLOSED;
                Dcc.setCV(OFFSET_POSITION_CV_VALUE + servoNum, CLOSED);
            }
        } else {
            if (servoStatus[servoNum].positionState != THROWN) {
                servoStatus[servoNum].moving = true;
                servoStatus[servoNum].positionState = THROWN;
                Dcc.setCV(OFFSET_POSITION_CV_VALUE + servoNum, THROWN);
            }
        }
    }
}

/*
 * Callback function to send ACK
 */
void notifyCVAck( void ) {

    // Set ACK pin for about 8ms
    digitalWrite(ACK_PIN, HIGH);
    delay(8);
    digitalWrite(ACK_PIN, LOW);

}

/*
 * Callback function used to log CV changes
 */
void notifyCVChange(uint16_t CV, uint8_t Value) {

#ifdef DEBUG
    Serial.print("CV "); Serial.print(CV, DEC); Serial.print(" changed, value (hex) : "); Serial.print(Value, HEX); Serial.println();
#endif

    // Check if is in resetting state to prevent double execution settings
    if (resettingState) {
        return;
    }

    // Change delay value
    if (CV == DELAY_CV_VALUE) {
#ifdef DEBUG
        Serial.println("Setting new delay value");
#endif
        setDelay();
    }

    // Change min position value (high bits)
    if (CV == MIN_HIGH_CV_VALUE) {
#ifdef DEBUG
        Serial.println("Setting new min value (high bits)");
#endif
        setServoValues();
        resetServoPosition(true, 0);
    }

    // Change min position value (low bits)
    if (CV == MIN_LOW_CV_VALUE) {
#ifdef DEBUG
        Serial.println("Setting new min value (low bits)");
#endif
        setServoValues();
        resetServoPosition(true, 0);
    }

    // Change max position value (high bits)
    if (CV == MAX_HIGH_CV_VALUE) {
#ifdef DEBUG
        Serial.println("Setting new max value (high bits)");
#endif
        setServoValues();
        resetServoPosition(true, 0);
    }

    // Change max position value (low bits)
    if (CV == MAX_LOW_CV_VALUE) {
#ifdef DEBUG
        Serial.println("Setting new max value (low bits)");
#endif
        setServoValues();
        resetServoPosition(true, 0);
    }

    // Change reversed mode
    if (CV >= OFFSET_REVERSED_CV_VALUE && CV < OFFSET_REVERSED_CV_VALUE + NUM_SERVOS) {
#ifdef DEBUG
        Serial.println("Setting new reversed mode");
#endif
        ServoReversedMode newReversedMode = (Value == 0 ? DISABLED : ENABLED);
        int servoNum = CV - OFFSET_REVERSED_CV_VALUE;
        resetServoPosition(false, servoNum);
    }

#ifdef DEBUG
    // Change servo state, only log received command
    if (CV >= OFFSET_POSITION_CV_VALUE && CV < OFFSET_POSITION_CV_VALUE + NUM_SERVOS) {
        Serial.print("Saving servo position state: "); Serial.println(Value == 0 ? "Closed" : "Thrown");
    }
#endif
}

/*
 * Callback function to reset CVs & parameters to default values
 */
void notifyCVResetFactoryDefault() {

#ifdef DEBUG
    Serial.println("Factory reset!");
#endif

    // Activate reset state
    resettingState = true;

    // Reset movements servo state
    for (int idx = 0; idx < NUM_SERVOS; idx++) {
        servoStatus[idx].moving = false;
    }

    // Reset DCC accessory address to 1
    Dcc.setCV(1, 1);
    Dcc.setCV(9, 0);

    // Reset delay, min & max servo values
    Dcc.setCV(DELAY_CV_VALUE,    DELAY_DEFAULT_VALUE);
    Dcc.setCV(MIN_HIGH_CV_VALUE, MIN_HIGH_DEFAULT_VALUE);
    Dcc.setCV(MIN_LOW_CV_VALUE,  MIN_LOW_DEFAULT_VALUE);
    Dcc.setCV(MAX_HIGH_CV_VALUE, MAX_HIGH_DEFAULT_VALUE);
    Dcc.setCV(MAX_LOW_CV_VALUE,  MAX_LOW_DEFAULT_VALUE);

    // Reset reversed mode parameters
    for (int idx = OFFSET_REVERSED_CV_VALUE; idx < OFFSET_REVERSED_CV_VALUE + NUM_SERVOS; idx++) {
        Dcc.setCV(idx, DISABLED);
    }

    // Reset servos position state
    for (int idx = OFFSET_POSITION_CV_VALUE; idx < OFFSET_POSITION_CV_VALUE + NUM_SERVOS; idx++) {
        Dcc.setCV(idx, CLOSED);
    }

    // Init updated delay, min & max servo position values
    setDelay();
    setServoValues();

    // Deactivate reset state
    resettingState = false;
}