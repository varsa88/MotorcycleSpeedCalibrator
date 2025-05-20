/* 
 * Project: Arduino Square Wave Modifier
 * Author: Daniel Ohlsson
 * 
 * Description:
 * - Use a PC901V (6-pin) optocoupler to safely read a 12V open-collector speed sensor signal
 * - Isolate and level-shift signal to 5V logic for Arduino
 * - Drive an output signal back to the vehicle. Optionally using a second PC901V
 * 
 * +-------------------+        +-------------------+        +----------------+
 * |  Vehicle Sensor   | -----> |    PC901V Input   | -----> |     Arduino     |
 * | (5V Square Wave)  |        |   (Isolated Side) |        |   (5V logic)    |
 * +-------------------+        +-------------------+        +----------------+
 * 
 * Cable Colors:
 * RED   = +12V (Sensor Power from vehicle)
 * BLACK = GND (Sensor Ground from vehicle)
 * WHITE = Signal from sensor (open-collector NPN, pulls low when active)
 * BLUE  = Optional signal back to vehicle (from Arduino D9)
 * 
 * INPUT STAGE: Sensor → Optocoupler → Arduino
 * --------------------------------------------
 *
 *          +12V VEHICLE SUPPLY
 *               |
 *           +---+----------------------+
 *           |                          |
 *        [RED]                      [BLACK]
 *      Sensor +12V              Sensor GND
 *           |                          |
 *           |                      +---+---------------------+
 *           |                      |                         |
 *           |                      |                     Arduino GND
 *           |                      |
 *           |                +-----+--------+
 *           |                |              |
 *           |             [WHITE]        PC901V (6-pin)
 *           |          Sensor Output     (Input Side)
 *           |                |            Pins 1 (Anode), 2 (Cathode)
 *           |                +------------+ Pin 2 (–)
 *           |                             |
 *           |                     +5V (Arduino)
 *           |                             |
 *           +----------[1kΩ]--------------+ Pin 1 (+)
 *                             
 * OUTPUT SIDE: PC901V → Arduino
 * ------------------------------
 * PC901V Pins:
 *   - Pin 3: NC
 *   - Pin 4: Emitter → Arduino GND
 *   - Pin 5: Collector → Arduino D2 (INT input)
 *                        |
 *                        +--[10kΩ or 12kΩ]--→ +5V (pull-up resistor)
 *                        +--||-------→ GND (optional 0.01 µF cap)
 *                               C1: Noise filter (optional)
 * 
 * f_c = 1 / (2 * π * R * C) ≈ 1 / (2 * π * 12,000 * 6.8e-9) ≈ 1.95 kHz
 * 
 * Result:
 * - When sensor is active (LOW), opto output transistor turns ON
 * - Arduino D2 sees LOW when sensor is active
 * - Arduino D2 sees HIGH when sensor is inactive (due to pull-up)
 * 
 * 
 * OPTIONAL OUTPUT STAGE: Arduino → Opto → Vehicle Input
 * -------------------------------------------------------
 * Use second PC901V (6-pin) to send signal from Arduino (D9) to vehicle side
 * 
 * Arduino D9
 *     |
 *    [330Ω]
 *     |
 *     +----> PC901V #2 Pin 1 (Anode)
 *           Pin 2 (Cathode) ---> Arduino GND
 *
 * PC901V #2 Output:
 *   - Pin 3: NC
 *   - Pin 4: Emitter → Vehicle GND
 *   - Pin 5: Collector → Vehicle input
 *                            |
 *                            +--[10kΩ]--→ +5V (vehicle side)
 *                            +--||-------→ GND (0.1 µF cap for decoupling)
 *                                   C2: Optional noise filter
 *
 * Pin 3 on output side) can be left unconnected
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

#include <Arduino.h>
#include <TimerOne.h>
#include <SimpleTimer.h>
#include <EEPROM.h> // Include EEPROM library

#define EEPROM_PERCENTAGE_ADDR 0 // EEPROM address to store the percentage
#define EEPROM_ALPHA_ADAPTIVE_ADDR 1 // EEPROM address to store the adaptive alpha state

// Add EEPROM addresses for new parameters
#define EEPROM_MIN_PERIOD_ADDR 2
#define EEPROM_MAX_PERIOD_ADDR 3
#define EEPROM_SMOOTHING_FACTOR_MIN_ADDR 4
#define EEPROM_SMOOTHING_FACTOR_MAX_ADDR 5

#define DEFAULT_PERCENTAGE 43.3 // Default percentage

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

// Constants
#define SQUAREWAVE 512u // Duty cycle for square wave
#define MIN_PERIOD 1800u // Minimum period in microseconds, 1800=0.0018s=555Hz
#define MAX_PERIOD 330000u // Maximum period in microseconds, 330000=0.33s=3Hz
#define TIMEOUT_PERIOD MAX_PERIOD // Timeout period in microseconds, same as maxPeriod

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

// Volatile variables for ISR
volatile unsigned long lastTime, delta_inraw, delta_in, delta_out;
volatile boolean EnableOutput = false;
volatile unsigned int counter1, counter2;
signed int percentage;
boolean debugactive = false; // Flag to indicate if debugging is activated

// MedianFilter class definition
class MedianFilter {
private:
    unsigned int size; // Size of the buffer
    unsigned long *buffer; // Pointer to the buffer array
    unsigned int index; // Current index in the buffer
    bool filled; // Flag to indicate if the buffer is filled

public:
    // Constructor
    MedianFilter(unsigned int bufferSize) : size(bufferSize), index(0), filled(false) {
        buffer = new unsigned long[size]; // Allocate memory for the buffer
        for (unsigned int i = 0; i < size; i++) {
            buffer[i] = 0; // Initialize buffer values to 0
        }
    }

    // Destructor
    ~MedianFilter() {
        delete[] buffer; // Free the allocated memory
    }

    // Add a new value to the buffer
    void addValue(unsigned long value) {
        buffer[index] = value; // Add the value to the current index
        index = (index + 1) % size; // Increment the index and wrap around if necessary
        if (index == 0) {
            filled = true; // Mark the buffer as filled after one full cycle
        }
    }

    // Get the median value from the buffer
    unsigned long getMedian() {
        unsigned int count = filled ? size : index; // Determine the number of valid values
        unsigned long *sortedBuffer = new unsigned long[count]; // Create a temporary array for sorting
        for (unsigned int i = 0; i < count; i++) {
            sortedBuffer[i] = buffer[i]; // Copy values to the temporary array
        }
        // Sort the temporary array
        for (unsigned int i = 0; i < count - 1; i++) {
            for (unsigned int j = i + 1; j < count; j++) {
                if (sortedBuffer[i] > sortedBuffer[j]) {
                    unsigned long temp = sortedBuffer[i];
                    sortedBuffer[i] = sortedBuffer[j];
                    sortedBuffer[j] = temp;
                }
            }
        }
        unsigned long median = sortedBuffer[count / 2]; // Get the median value
        delete[] sortedBuffer; // Free the temporary array
        return median;
    }
};

// Declare adaptiveAlphaEnabled as a global variable
bool adaptiveAlphaEnabled = true; // Default to enabled

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
// Instantiate the MedianFilter with a buffer size of 5
MedianFilter medianFilter(5);
// Instantiate the ExponentialFilter with the default smoothing factor
ExponentialFilter exponentialFilter(SMOOTHING_FACTOR);

// Declare variables for the parameters
unsigned long minPeriod = MIN_PERIOD;
unsigned long maxPeriod = MAX_PERIOD;
unsigned int smoothingFactorMin = SMOOTHING_FACTOR_MIN;
unsigned int smoothingFactorMax = SMOOTHING_FACTOR_MAX;

// Add global variables to track debug information
unsigned long totalTimeouts = 0;  // Total number of timeouts since startup

// Add a global variable to track the difference in delta_inraw between consecutive measurements
long deltaInRawDifference = 0;

// Maximum value of long type
#define LONG_MAX 2147483647

long maximumdeltaInRawDifference = 0; // Maximum delta_inraw difference
long minimumdeltaInRawDifference = LONG_MAX; // Minimum delta_inraw difference

// Function declarations
void hall_ISR();
void slow_1Hz();
void fast_100Hz();
void fast_500Hz();
void checkTimeout();
void updateFilter();
void calculateOutputPeriod();
void generateSquareWaveOutput();
void handleSerialMenu();
void calculateDebugInfo();
void serialPrint();

void setup() {
    Serial.begin(9600);
    //DEBUG_SERIAL.begin(9600);
    #if (USE_SERIAL)
    while (!Serial || millis() < 3000UL); // hangs here 5 seconds if no USB
    #endif
    
    timer.setInterval(1000u, slow_1Hz);
    timer.setInterval(10u, fast_100Hz);
    timer.setInterval(2u, fast_500Hz);
    
    pinMode(LED_BUILTIN, OUTPUT); // LED pin
    pinMode(INPUT_PIN, INPUT); // Hall effect sensor pin using external pull-up resistor
    pinMode(OUTPUT_PIN, OUTPUT); // Square-wave output pin
    
    // Load percentage from EEPROM
    EEPROM.get(EEPROM_PERCENTAGE_ADDR, percentage);
    if (percentage < 0 || percentage > 200 * PERCENTAGE_SCALE) {
        percentage = DEFAULT_PERCENTAGE * PERCENTAGE_SCALE; // Reset to default if invalid
    }

    // Load adaptive alpha state from EEPROM
    EEPROM.get(EEPROM_ALPHA_ADAPTIVE_ADDR, adaptiveAlphaEnabled);

    // Load parameters from EEPROM
    EEPROM.get(EEPROM_MIN_PERIOD_ADDR, minPeriod);
    EEPROM.get(EEPROM_MAX_PERIOD_ADDR, maxPeriod);
    EEPROM.get(EEPROM_SMOOTHING_FACTOR_MIN_ADDR, smoothingFactorMin);
    EEPROM.get(EEPROM_SMOOTHING_FACTOR_MAX_ADDR, smoothingFactorMax);

    // Validate loaded values
    if (minPeriod < 100 || minPeriod > 1000000) minPeriod = MIN_PERIOD;
    if (maxPeriod < 100 || maxPeriod > 1000000) maxPeriod = MAX_PERIOD;
    if (smoothingFactorMin < 1 || smoothingFactorMin > 100) smoothingFactorMin = SMOOTHING_FACTOR_MIN;
    if (smoothingFactorMax < 1 || smoothingFactorMax > 100) smoothingFactorMax = SMOOTHING_FACTOR_MAX;

    Serial.println("Type 'h' for serial menu.");
    Timer1.initialize(1000u); // Initialize Timer1, 1000us = 1ms
    Timer1.disablePwm(OUTPUT_PIN);
    digitalWrite(OUTPUT_PIN, HIGH);

    // Initialize the encoder filter object
    exponentialFilter = ExponentialFilter(SMOOTHING_FACTOR);

    lastTime = micros(); // Initiate lastTime
    attachInterrupt(digitalPinToInterrupt(INPUT_PIN), hall_ISR, FALLING); // Attach interrupt
}

void loop() {
    timer.run(); // Run the timer
    handleSerialMenu(); // Check for serial input
}

void hall_ISR() {
    unsigned long currentTime = micros();
    delta_inraw = currentTime - lastTime;
    lastTime = currentTime;
}

void checkTimeout() {
    unsigned long timeSinceLastInterrupt = micros() - lastTime;
    if (timeSinceLastInterrupt >= TIMEOUT_PERIOD) {
        if (!EnableOutput) {
            totalTimeouts++; // Increment the timeout counter if a new timeout occurs
        }
        EnableOutput = false;
    } else {
        EnableOutput = true;
    }
}

void updateFilter() {
    if (delta_inraw > MIN_PERIOD) {
        // Saturate to make sure the filter is not updated with insane values
        unsigned long delta = SATURATE(delta_inraw, MIN_PERIOD, MAX_PERIOD);

        // Add the raw value to the median filter
        medianFilter.addValue(delta);

        // Get the median value and pass it to the exponential filter
        unsigned long medianValue = medianFilter.getMedian();
        exponentialFilter.update(medianValue);
    }
}

void calculateOutputPeriod() {
    delta_in = exponentialFilter.getValue();
    delta_out = (delta_in * PERCENTAGE_HUNDRED) / percentage;

    if (adaptiveAlphaEnabled) {
        // Calculate alpha value based on delta using linear interpolation
        unsigned int newAlpha = SMOOTHING_FACTOR_MAX - ((delta_in - MIN_PERIOD) * (SMOOTHING_FACTOR_MAX - SMOOTHING_FACTOR_MIN)) / (MAX_PERIOD - MIN_PERIOD);
        exponentialFilter.setAlpha(newAlpha);
    }
}

void generateSquareWaveOutput() {
    if (EnableOutput == true) {
        Timer1.pwm(OUTPUT_PIN, SQUAREWAVE);
        Timer1.setPeriod(delta_out);
    } else {
        Timer1.disablePwm(OUTPUT_PIN);
        digitalWrite(OUTPUT_PIN, HIGH);
    }
}

void slow_1Hz() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (DEBUG_INFORMATION && debugactive) serialPrint();
}

void fast_100Hz() {
    calculateOutputPeriod();
    checkTimeout();
    generateSquareWaveOutput();
}

void fast_500Hz() {
    updateFilter();
    calculateDebugInfo();
}

// Update the handleSerialMenu function to include the new menu option
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

            case 'm': // Set minPeriod
                Serial.print("Current MIN_PERIOD: ");
                Serial.println(minPeriod);
                Serial.println("Enter new MIN_PERIOD (100-1000000):");
                while (!Serial.available());
                minPeriod = Serial.parseInt();
                if (minPeriod >= 100 && minPeriod <= 1000000) {
                    EEPROM.put(EEPROM_MIN_PERIOD_ADDR, minPeriod);
                    Serial.print("MIN_PERIOD updated to: ");
                    Serial.println(minPeriod);
                } else {
                    Serial.println("Invalid MIN_PERIOD. Must be between 100 and 1000000.");
                }
                break;

            case 'x': // Set maxPeriod
                Serial.print("Current MAX_PERIOD: ");
                Serial.println(maxPeriod);
                Serial.println("Enter new MAX_PERIOD (100-1000000):");
                while (!Serial.available());
                maxPeriod = Serial.parseInt();
                if (maxPeriod >= 100 && maxPeriod <= 1000000) {
                    EEPROM.put(EEPROM_MAX_PERIOD_ADDR, maxPeriod);
                    Serial.print("MAX_PERIOD updated to: ");
                    Serial.println(maxPeriod);
                } else {
                    Serial.println("Invalid MAX_PERIOD. Must be between 100 and 1000000.");
                }
                break;

            case 'n': // Set smoothingFactorMin
                Serial.print("Current SMOOTHING_FACTOR_MIN: ");
                Serial.println(smoothingFactorMin);
                Serial.println("Enter new SMOOTHING_FACTOR_MIN (1-100):");
                while (!Serial.available());
                smoothingFactorMin = Serial.parseInt();
                if (smoothingFactorMin >= 1 && smoothingFactorMin <= 100) {
                    EEPROM.put(EEPROM_SMOOTHING_FACTOR_MIN_ADDR, smoothingFactorMin);
                    Serial.print("SMOOTHING_FACTOR_MIN updated to: ");
                    Serial.println(smoothingFactorMin);
                } else {
                    Serial.println("Invalid SMOOTHING_FACTOR_MIN. Must be between 1 and 100.");
                }
                break;

            case 'f': // Set smoothingFactorMax
                Serial.print("Current SMOOTHING_FACTOR_MAX: ");
                Serial.println(smoothingFactorMax);
                Serial.println("Enter new SMOOTHING_FACTOR_MAX (1-100):");
                while (!Serial.available());
                smoothingFactorMax = Serial.parseInt();
                if (smoothingFactorMax >= 1 && smoothingFactorMax <= 100) {
                    EEPROM.put(EEPROM_SMOOTHING_FACTOR_MAX_ADDR, smoothingFactorMax);
                    Serial.print("SMOOTHING_FACTOR_MAX updated to: ");
                    Serial.println(smoothingFactorMax);
                } else {
                    Serial.println("Invalid SMOOTHING_FACTOR_MAX. Must be between 1 and 100.");
                }
                break;

            case 'h': // Help menu
                Serial.println("Serial Menu:");
                Serial.println("s - Set percentage");
                Serial.println("r - Read current percentage");
                Serial.println("m - Set MIN_PERIOD");
                Serial.println("x - Set MAX_PERIOD");
                Serial.println("n - Set SMOOTHING_FACTOR_MIN");
                Serial.println("f - Set SMOOTHING_FACTOR_MAX");
                Serial.println("h - Show this menu");
                Serial.println("d - Toggle debug mode");
                Serial.println("a - Toggle adaptive alpha");
                break;

            case 'd': // Toggle debug mode
                debugactive = !debugactive; // Toggle debug mode
                if (debugactive) {
                    Serial.println("Debugging activated.");
                } else {
                    Serial.println("Debugging deactivated.");
                }
                break;

            case 'a': // Toggle adaptive alpha
                adaptiveAlphaEnabled = !adaptiveAlphaEnabled; // Toggle the state
                EEPROM.put(EEPROM_ALPHA_ADAPTIVE_ADDR, adaptiveAlphaEnabled);
                if (adaptiveAlphaEnabled) {
                    Serial.println("Adaptive alpha enabled.");
                } else {
                    Serial.println("Adaptive alpha disabled.");
                }
                break;

            default:
                Serial.println("Invalid command. Type 'h' for help.");
                break;
        }
    }
}

void calculateDebugInfo() {
    if ((delta_inraw > MIN_PERIOD) && (delta_inraw < MAX_PERIOD)) {
        // Calculate the difference in delta_inraw between consecutive measurements
        deltaInRawDifference = delta_inraw - lastTime;

        // Update the maximum and minimum delta_inraw difference
        if (deltaInRawDifference > maximumdeltaInRawDifference) {
            maximumdeltaInRawDifference = deltaInRawDifference;
        }
        if (deltaInRawDifference < minimumdeltaInRawDifference) {
            minimumdeltaInRawDifference = deltaInRawDifference;
        }
    }
}

void serialPrint()
{
    Serial.print("Delta in raw: ");
    Serial.print(delta_inraw);
    Serial.print(" us, Delta in filtered: ");
    Serial.print(delta_in);
    Serial.print(" us, Delta out: ");
    Serial.print(delta_out);
    //Serial.print(" us, Percentage: ");
    //Serial.print(percentage);
    //Serial.println(" deci%");

    Serial.print("Delta difference: ");
    Serial.println(deltaInRawDifference);
    Serial.print("Max delta difference: ");
    Serial.println(maximumdeltaInRawDifference);
    Serial.print("Min delta difference: ");
    Serial.println(minimumdeltaInRawDifference);
    Serial.print("Total timeouts: ");
    Serial.println(totalTimeouts);

    #if (DEBUG_ERROR)
    Serial.print("Counter1: ");
    Serial.print(counter1);
    Serial.print(", Counter2: ");
    Serial.println(counter2);
    #endif
}