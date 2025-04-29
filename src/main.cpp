/* Project: Arduino Square Wave Modifier
 * Author: Daniel Ohlsson
 * 
 * +-------------------+        +-------------------+        +----------------+
 * |  Vehicle Sensor   | -----> |    PC901V Input   | -----> |     Arduino     |
 * | (5V Square Wave)  |        |   (Isolated Side) |        |   (5V logic)    |
 * +-------------------+        +-------------------+        +----------------+
 * 
 * Cable Colors:
 * RED   = +5V (Power supply from the vehicle's sensor system)
 * BLACK = GND (Ground connection shared with the vehicle's sensor system)
 * BLUE = Signal Output (processed signal from Arduino to the vehicle)
 * WHITE  = Signal Input (raw signal from the vehicle's sensor to Arduino)
 * 
 * SENSOR SIDE (5V SYSTEM)
 * 
 * +5V (from sensor system)
 *   |
 *   R1 (330Ω)
 *   |
 *   |-----> Pin 1 (Anode) [PC901V]
 *           Pin 2 (Cathode) ---> GND (sensor side)
 * 
 * ARDUINO SIDE (ISOLATED LOGIC SIDE)
 * 
 * Pin 6 (Vcc) -------> +5V (Arduino)
 * Pin 5 (GND) -------> GND (Arduino)
 *                      |
 *                      C1 (0.1 µF ceramic)
 *                      |
 *                      Vcc ---||--- GND   [decoupling capacitor]
 * 
 * Pin 4 (Vo) ---------> Arduino input pin (e.g., D2)
 *                      |
 *                      R2 (10kΩ pull-up to 5V Arduino)
 * 
 * OPTIONAL: Noise filter across LED side:
 * C2 (100nF ceramic)
 * Connected across Pin 1 ↔ Pin 2
 * 
 * NC PINS:
 * Pin 3 → No connection
 */

 /* Optional opto-isolator circuit for Arduino output control:
 *
 * ARDUINO SIDE (OUTPUT CONTROL)
 *
 * Arduino output pin (e.g., D3)
 *      |
 *      R3 (330Ω)
 *      |
 *      |------> Pin 1 (Anode) [PC901V #2]
 *               Pin 2 (Cathode) ---> GND (Arduino)
 *
 *VEHICLE SIDE (ISOLATED OUTPUT)
 *
 * Pin 6 (Vcc) -------> +5V (vehicle system)
 * Pin 5 (GND) -------> GND (vehicle system)
 *                      |
 *                      C3 (0.1 µF ceramic cap)
 *                      |
 *                      Vcc ---||--- GND   [decoupling capacitor]
 *
 * Pin 4 (Vo) ---------> Vehicle input signal wire
 *                      |
 *                      R4 (10kΩ pull-up to 5V vehicle system)
 *
 *NC:
 * Pin 3 → No connection
 */

/* 
 * 18 inch rear wheel circumference , pi*18inch = 56.5486672 inch= 1.437m
 * 250kph = 250000m/3600s = 69.4444m/s
 * 69.4444m/s / 1.437m = 48.4Hz
 * Front sprocket = 14T, rear sprocket = 50T
 * 17/45 = 0.37
 * 48.4Hz / 0.37 = 130.1Hz
 * 130.1Hz * 4 = 520.4Hz
 * 1 / 520.4Hz = 0.00192s = 1920us
 */

/* PIN Setup
PD0 = 0
PD1 = 1
PD2 = 2
PD3 = 3
PD4 = 4
PD5 = 5
PD6 = 6
PD7 = 7

PB0 = 8
PB1 = 9
PB2 = 10
PB3 = 11
PB4 = 12
PB5 = 13
*/

#include <Arduino.h>
#include <TimerOne.h>
#include <SimpleTimer.h>
#include <EEPROM.h> // Include EEPROM library

#define EEPROM_PERCENTAGE_ADDR 0 // EEPROM address to store the percentage

//#define DEFAULT_PERCENTAGE 86.5 // Default percentage if not using dip switches
#define DEFAULT_PERCENTAGE 100.0 // Default percentage if not using dip switches
//#define USE_DIPSWITCHES // Uncomment to use dip switches for percentage setting

// Macros for convenience
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define SATURATE(x, min, max) (MAX(MIN((x), (max)), (min)))

// Debugging macros
#define DEBUG_ERROR false
#define DEBUG_ERROR_SERIAL if (DEBUG_ERROR) Serial

#define DEBUG_WARNING false
#define DEBUG_WARNING_SERIAL if (DEBUG_WARNING) Serial

#define DEBUG_INFORMATION true
#define DEBUG_INFORMATION_SERIAL if (DEBUG_INFORMATION) Serial

#if (((DEBUG_ERROR) || (DEBUG_WARNING)) || (DEBUG_INFORMATION))
#define USE_SERIAL true
#else
#define USE_SERIAL false
#endif
#define DEBUG_SERIAL if (USE_SERIAL) Serial

// Pin definitions
#define INPUT_PIN 2
#define OUTPUT_PIN 9 // Pin for square wave output (OC1A) must be OC1A or OC1B for Timer1, Pin 9 or 10 on Arduino Uno
#define LED_BUILTIN 13 // Built-in LED pin

#ifdef USE_DIPSWITCHES
#define DIRECTION_PIN 4
#endif

// Constants
#define SQUAREWAVE 512u // Duty cycle for square wave
#define DEBOUNCE_US 400u // Debounce time in microseconds, 400=0.0004s=2500Hz
#define MIN_PERIOD 1800u // Minimum period in microseconds, 1800=0.0018s=555Hz
#define MAX_PERIOD 330000u // Maximum period in microseconds, 330000=0.33s=3Hz
#define TIMEOUT_PERIOD MAX_PERIOD
//#define TIMEOUT_PERIOD (3u * MAX_PERIOD)

#define SCALE_DECI 10
#define SCALE_CENTI 100
#define PERCENTAGE_SCALE SCALE_DECI
#define PERCENTAGE_MULTIPLIER 5
#define PERCENTAGE_DIVIDER 10
#define PERCENTAGE_HUNDRED (100 * PERCENTAGE_SCALE)

// Define the smoothing factor
#define SMOOTHING_FACTOR_MIN 2
#define SMOOTHING_FACTOR 5
#define SMOOTHING_FACTOR_MAX 20
//#define USE_ALPHA_ADAPTIVE // Uncomment to use adaptive alpha value based on delta_in

// Volatile variables for ISR
volatile unsigned long lastTime, delta_inraw, delta_in, delta_out;
volatile boolean EnableOutput = false;
volatile unsigned int counter1, counter2;
signed int percentage;
boolean debugactive = false; // Flag to indicate if debugging is activated

// ExponentialFilter class definition
class ExponentialFilter {
private:
    unsigned int alpha; // Smoothing factor (1 <= alpha <= 100)
    unsigned long filteredValue = MAX_PERIOD;

public:
    ExponentialFilter(unsigned int smoothingFactor)
        : alpha(smoothingFactor <= 100 ? smoothingFactor : 100), filteredValue(0) {}

    void update(unsigned long newValue) {
        filteredValue = (alpha * newValue + (100 - alpha) * filteredValue) / 100;
    }

    unsigned long getValue() const {
        return filteredValue;
    }

    void setAlpha(unsigned int newAlpha) {
        alpha = (newAlpha <= 100 ? newAlpha : 100);
    }
};

// Instantiate objects
SimpleTimer timer;
ExponentialFilter encoderFilter(SMOOTHING_FACTOR);

// Function declarations
void hall_ISR();
void slow_1Hz();
#if (DEBUG_ERROR)
void fast_100Hz();
#endif
void calculatePercentage();
void checkTimeout();
void updateFilter();
void calculateOutputPeriod();
void generateSquareWaveOutput();
void handleSerialMenu();
void serialPrint();

void setup() {
    Serial.begin(9600);
    //DEBUG_SERIAL.begin(9600);
    #if (USE_SERIAL)
    while (!Serial || millis() < 3000UL); // hangs here 5 seconds if no USB
    #endif
    
    timer.setInterval(1000u, slow_1Hz);
    #if (DEBUG_ERROR)
    timer.setInterval(10u, fast_100Hz);
    #endif
    
    pinMode(LED_BUILTIN, OUTPUT); // LED pin
    pinMode(INPUT_PIN, INPUT); // Hall effect sensor pin using external pull-up resistor
    pinMode(OUTPUT_PIN, OUTPUT); // Square-wave output pin
    #ifdef USE_DIPSWITCHES
    pinMode(DIRECTION_PIN, INPUT_PULLUP); // Direction pin HIGH means negative
    
    /* Dip-switches */
    // pinMode(5, INPUT_PULLUP); /* bit0 */
    pinMode(5, INPUT_PULLUP); /* bit1 */
    pinMode(6, INPUT_PULLUP); /* bit2 */
    pinMode(7, INPUT_PULLUP); /* bit3 */
    pinMode(8, INPUT_PULLUP); /* bit4 */
    pinMode(10, INPUT_PULLUP); /* bit5 */
    pinMode(11, INPUT_PULLUP); /* bit6 */
    pinMode(12, INPUT_PULLUP); /* bit7 */
    #endif
    
    
    // Load percentage from EEPROM
    EEPROM.get(EEPROM_PERCENTAGE_ADDR, percentage);
    if (percentage < 0 || percentage > 200 * PERCENTAGE_SCALE) {
        percentage = DEFAULT_PERCENTAGE * PERCENTAGE_SCALE; // Reset to default if invalid
    }

    Serial.println("Type 'h' for serial menu.");
    //calculatePercentage();

    Timer1.initialize(1000u); // Initialize Timer1, 1000us = 1ms
    Timer1.disablePwm(OUTPUT_PIN);
    digitalWrite(OUTPUT_PIN, HIGH);

    // Initialize the encoder filter object
    encoderFilter = ExponentialFilter(SMOOTHING_FACTOR);

    lastTime = micros(); // Initiate lastTime
    attachInterrupt(digitalPinToInterrupt(INPUT_PIN), hall_ISR, FALLING); // Attach interrupt
}

void calculatePercentage() {
    #ifdef USE_DIPSWITCHES
    signed int calc;
    byte dipswitch = 0b10000000;
    dipswitch |= (PIND & 0b11100000) >> 5;
    dipswitch |= (PINB & 0b00011100) << 2;
    dipswitch |= (PINB & 0b00000001) << 3;
    dipswitch = ~dipswitch;

    if (digitalRead(DIRECTION_PIN) == HIGH) {
        calc = (dipswitch * (-1));
    } else {
        calc = dipswitch;
    }
    calc *= PERCENTAGE_SCALE;
    calc *= PERCENTAGE_MULTIPLIER;
    calc /= PERCENTAGE_DIVIDER;
    calc += PERCENTAGE_HUNDRED;
    percentage = calc;
    #else
    percentage = DEFAULT_PERCENTAGE * PERCENTAGE_SCALE; // Default percentage if not using dip switches
    #endif
}

void loop() {
    handleSerialMenu(); // Check for serial input
    timer.run();
    updateFilter();
    calculateOutputPeriod();
    checkTimeout();
    generateSquareWaveOutput();
}

void hall_ISR() {
    unsigned long currentTime = micros();
    delta_inraw = currentTime - lastTime;
    lastTime = currentTime;
}

void checkTimeout() {
    unsigned long timeSinceLastInterrupt = micros() - lastTime;
    EnableOutput = (timeSinceLastInterrupt < TIMEOUT_PERIOD ? true : false);
}

void updateFilter() {
    if (delta_inraw > MIN_PERIOD) {
        // Saturate to make sure the filter is not updated with insane values
        unsigned long delta = SATURATE(delta_inraw, MIN_PERIOD, MAX_PERIOD);
        // Update the filter with the raw encoder signal value
        encoderFilter.update(delta);
    }
}

void calculateOutputPeriod() {
    delta_in = encoderFilter.getValue();
    delta_out = (delta_in * PERCENTAGE_HUNDRED) / percentage;

    #ifdef USE_ALPHA_ADAPTIVE
    // Calculate alpha value based on delta using linear interpolation
     unsigned int newAlpha = SMOOTHING_FACTOR_MAX - ((delta_in - MIN_PERIOD) * (SMOOTHING_FACTOR_MAX - SMOOTHING_FACTOR_MIN)) / (MAX_PERIOD - MIN_PERIOD);
     encoderFilter.setAlpha(newAlpha);
    #endif
}

void generateSquareWaveOutput() {
    if (EnableOutput == true) {
        Timer1.pwm(OUTPUT_PIN, SQUAREWAVE);
        Timer1.setPeriod(delta_out);
    } else {
        if (DEBUG_ERROR) counter2++;
        Timer1.disablePwm(OUTPUT_PIN);
        digitalWrite(OUTPUT_PIN, HIGH);
    }
}

void slow_1Hz() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (DEBUG_INFORMATION && debugactive) serialPrint();
}

#if (DEBUG_ERROR)
void fast_100Hz() {
    unsigned long currentTime = micros();
    if (currentTime - lastTime > TIMEOUT_PERIOD) {
        if (DEBUG_ERROR) counter1++;
    }
}
#endif

void handleSerialMenu() {
    if (Serial.available()) {
        char command = Serial.read();
        command = tolower(command); // Convert command to lowercase
        int newPercentage = 0; // Declare the variable outside the switch statement
        switch (command) {
            case 's': // Set percentage
                Serial.println("Enter new decipercentage (0-2000):");
                while (!Serial.available());
                newPercentage = Serial.parseInt();
                if (newPercentage >= 0 && newPercentage <= 2000) {
                    percentage = newPercentage;
                    EEPROM.put(EEPROM_PERCENTAGE_ADDR, percentage);
                    Serial.print("Percentage updated to: ");
                    Serial.println(newPercentage);
                } else {
                    Serial.println("Invalid percentage. Must be between 0 and 200.");
                }
                break;

            case 'r': // Read current percentage
                Serial.print("Current decipercentage: ");
                Serial.println(percentage);
                break;

            case 'h': // Help menu
                Serial.println("Serial Menu:");
                Serial.println("s - Set percentage");
                Serial.println("r - Read current percentage");
                Serial.println("h - Show this menu");
                Serial.println("d - Toggle debug mode");
                break;

            case 'd': // Activate Help menu
                debugactive = !debugactive; // Toggle debug mode
                if (debugactive) {
                    Serial.println("Debugging activated.");
                } else {
                    Serial.println("Debugging deactivated.");
                }
                break;

            default:
                Serial.println("Invalid command. Type 'h' for help.");
                break;
        }
    }
}

void serialPrint()
{
    Serial.print("Delta in raw: ");
    Serial.print(delta_inraw);
    Serial.print("Delta in filtered: ");
    Serial.print(delta_in);
    Serial.print(" us, Delta out: ");
    Serial.print(delta_out);
    Serial.print(" us, Percentage: ");
    Serial.print(percentage);
    Serial.println("deci%");

    #if (DEBUG_ERROR)
    Serial.print("Counter1: ");
    Serial.print(counter1);
    Serial.print(", Counter2: ");
    Serial.println(counter2);
    #endif
}