#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Pixy2.h>
#include <SPI.h>


// Pin Definitions
#define PIXY_RX_PIN 0            // Pixy2 Camera RX for serial communication
#define PIXY_TX_PIN 1            // Pixy2 Camera TX for serial communication
#define MOTOR_AIN1 2             // Motor 1 PWM speed control
#define MOTOR_AIN2 3             // Motor 1 direction control
#define TEMP_PIN 12               // DS18B20 Temperature Sensor (Data pin)
#define MOTOR_BIN1 5             // Motor 2 PWM speed control
#define MOTOR_BIN2 6             // Motor 2 direction control
#define POWERBOOST_ENABLE 7      // PowerBoost ENABLE through transistor 2A2222A
#define LED_PIN 8                // APA106 LED for LED strip
#define RELAY_PIN 9              // Control Power Relay
#define FEEDBACK_LED 10          // LED for robot feedback
#define BATTERY_LBO 11           // this pin is not correct it is used for alerting the operator that the battery is less than 3.2v
#define MOTORS_SLEEP 4         // Motors Sleep/12
#define BATTERY_PIN A0           // Voltage Divider for battery level monitoring
#define PROXIMITY_SDA A4         // Proximity Sensor SDA (I2C communication)
#define PROXIMITY_SCL A5         // Proximity Sensor SCL (I2C communication)
#define CHARGING_PIN 13

// Battery and Temperature Thresholds
const float R1 = 3300.0;          // Resistor R1 (3.3kΩ)
const float R2 = 10000.0;         // Resistor R2 (10kΩ)
const float MAX_VOLTAGE = 4.2;    // Maximum allowable battery voltage
const float HIGH_TEMP_THRESHOLD = 45; // High temperature threshold (Celsius)
const float LOW_TEMP_THRESHOLD = 35;  // Cool-down temperature threshold (Celsius)
const float batteryMaxVoltage = 4.2;  // Maximum battery voltage
const float voltage50Percent = 3.7;   // Voltage at 50% charge
const float voltage30Percent = 3.5;   // Voltage at 30% charge
const float voltage10Percent = 3.3;   // Voltage at 10% charge

const int BATTERY_READINGS = 10;  // Number of readings for averaging
float batteryReadings[BATTERY_READINGS] = {0}; // Buffer for voltage readings
int readIndex = 0;                // Index for rolling average buffer

// LED Behavior Parameters
#define COLOR_GREEN strip.Color(0, 255, 0)
#define COLOR_YELLOW strip.Color(255, 255, 0)
#define COLOR_RED strip.Color(255, 0, 0)
#define COLOR_BROWN strip.Color(139, 69, 19)

// NeoPixel Strip Initialization
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);
// DS18B20 Sensor Initialization
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
Pixy2 pixy;

void updateLEDStatus(float batteryVoltage, bool charging, bool temperatureHigh = false);


void setup()
{
  Serial.begin(9600);
  

  strip.begin();                          // Initialize LED strip library
  strip.clear();                          // Clear the LED strip
  strip.show();                           // Update the LED strip
  Wire.begin();
  sensors.begin();
  pixy.init();
  Serial.println("Pixy2 Initialized");

  pinMode(MOTOR_AIN1, OUTPUT);   // PWM speed control
  pinMode(MOTOR_AIN2, OUTPUT);   // Direction control
  pinMode(MOTOR_BIN1, OUTPUT);   // PWM speed control
  pinMode(MOTOR_BIN2, OUTPUT);   // Direction control
  pinMode(TEMP_PIN, INPUT);      // DS18B20 data pin
  pinMode(POWERBOOST_ENABLE, OUTPUT);  // Enable PowerBoost via transistor
  pinMode(LED_PIN, OUTPUT);      // Data pin for APA106 LED strip
  pinMode(RELAY_PIN, OUTPUT);    // Control relay for charging
  pinMode(FEEDBACK_LED, OUTPUT); // General feedback LED
  pinMode(BATTERY_LBO, INPUT);   // PowerBoost battery monitoring (digital input)
  pinMode(MOTORS_SLEEP, OUTPUT); // Sleep pin to control motors' power state

  digitalWrite(POWERBOOST_ENABLE, HIGH);  // Turn on PowerBoost
  digitalWrite(MOTORS_SLEEP, HIGH);        // Put motors in sleep mode initially
  digitalWrite(RELAY_PIN, LOW);           // Ensure relay is off (charging inactive)


  Serial.print("Starting...\n");
  
}

// Main Loop
void loop() {
    // Measure battery voltage and check if temperature is high
    float battery_voltage = monitorBattery();
    float temperature = monitorTemperature();
    bool temperatureHigh = temperature >= HIGH_TEMP_THRESHOLD;

    // Logic to handle different states
    if (isCharging()) {
        // If charging, manage the charging process
        stopFunctions();
        chargingManagement(battery_voltage, temperature);
    } 
    else if (battery_voltage <= voltage10Percent) {
        // If battery voltage is critically low
        Serial.println("Battery voltage critically low! Stopping all functions.");
        stopFunctions(); // Stop all non-essential components
        updateLEDStatus(battery_voltage, false, temperatureHigh); // Update LED for critical state
    } 
    else {
        // Normal operation
        updateLEDStatus(battery_voltage, false, temperatureHigh); // Update LED in normal operation mode
        enableFunctions();

        // Perform robot operations
        // Move Forward
        analogWrite(MOTOR_AIN1, 200);
        analogWrite(MOTOR_AIN2, 0);
        analogWrite(MOTOR_BIN1, 200);
        analogWrite(MOTOR_BIN2, 0);
        delay(1000);

        // Turn right
        analogWrite(MOTOR_AIN1, 0);
        analogWrite(MOTOR_AIN2, 200);
        analogWrite(MOTOR_BIN1, 200);
        analogWrite(MOTOR_BIN2, 0);
        delay(1000);

        // Move backward
        analogWrite(MOTOR_AIN1, 0);
        analogWrite(MOTOR_AIN2, 200);
        analogWrite(MOTOR_BIN1, 0);
        analogWrite(MOTOR_BIN2, 200);
        delay(1000);

        // Turn left
        analogWrite(MOTOR_AIN1, 200);
        analogWrite(MOTOR_AIN2, 0);
        analogWrite(MOTOR_BIN1, 0);
        analogWrite(MOTOR_BIN2, 200);
        delay(1000);

        // Pixy2 Block Detection
        int i;
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks) {
            Serial.print("Detected ");
            Serial.println(pixy.ccc.numBlocks);
            for (i = 0; i < pixy.ccc.numBlocks; i++) {
                Serial.print("  block ");
                Serial.print(i);
                Serial.print(": ");
                pixy.ccc.blocks[i].print();
            }
        }
    }

    // Delay before next loop iteration
    delay(1000);
}





// Function to monitor battery voltage with smoothing
float monitorBattery() {
    int sensorValue = analogRead(BATTERY_PIN);
    float voltage = sensorValue * (3.3 / 1023.0); // ADC to voltage conversion
    float batteryVoltage = voltage * (R1 + R2) / R2; // Voltage divider formula

    // Update rolling average buffer
    batteryReadings[readIndex] = batteryVoltage;
    readIndex = (readIndex + 1) % BATTERY_READINGS;

    // Calculate average voltage
    float averageVoltage = 0;
    for (int i = 0; i < BATTERY_READINGS; i++) {
        averageVoltage += batteryReadings[i];
    }
    averageVoltage /= BATTERY_READINGS;

    Serial.print("Battery Voltage: ");
    Serial.println(averageVoltage);
    return averageVoltage;
}

// Function to monitor temperature using the DS18B20 sensor
float monitorTemperature() {
    sensors.requestTemperatures(); 
    float temperature = sensors.getTempCByIndex(0);
    Serial.print("Temperature: ");
    Serial.println(temperature);
    return temperature;
}

// Function to update LED status based on battery voltage and robot state
void updateLEDStatus(float batteryVoltage, bool charging, bool temperatureHigh) {
    static unsigned long previousMillis = 0;
    const unsigned long interval = 500; // Flash interval in milliseconds
    unsigned long currentMillis = millis();

    // Charging Mode LED Behavior
    if (charging) {
        if (temperatureHigh) {
            // Flash orange if temperature is too high
            if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                static bool ledOn = false;
                ledOn = !ledOn;
                if (ledOn) {
                    strip.setPixelColor(0, strip.Color(255, 165, 0));  // Flash Orange for high temperature
                } else {
                    strip.clear();
                }
                strip.show();
            }
        } else if (batteryVoltage >= batteryMaxVoltage) {
            // Solid blue when fully charged
            strip.setPixelColor(0, strip.Color(0, 0, 255));  // Solid Blue for fully charged
            strip.show();
        } else {
            // Flash based on battery voltage level
            if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                static bool ledOn = false;
                ledOn = !ledOn;

                if (ledOn) {
                    if (batteryVoltage >= voltage50Percent) {
                        strip.setPixelColor(0, COLOR_GREEN);  // Flash Green above 50%
                    } else if (batteryVoltage >= voltage30Percent) {
                        strip.setPixelColor(0, COLOR_YELLOW);  // Flash Yellow between 30%-50%
                    } else {
                        strip.setPixelColor(0, COLOR_RED);  // Flash Red below 30%
                    }
                } else {
                    strip.clear();
                }
                strip.show();
            }
        }
    }
    // Normal Operation LED Behavior
    else {
        if (batteryVoltage >= voltage50Percent) {
            strip.setPixelColor(0, COLOR_GREEN);  // Solid Green above 50%
        } else if (batteryVoltage >= voltage30Percent) {
            strip.setPixelColor(0, COLOR_YELLOW);  // Solid Yellow between 30%-50%
        } else if (batteryVoltage <= voltage30Percent) {
            strip.setPixelColor(0, COLOR_RED);  // Solid Red for critical charge
        }
        strip.show();
    }
}

// Function to check if the robot is charging
bool isCharging() {
    int lboStatus = digitalRead(CHARGING_PIN);
    return lboStatus == HIGH;
}
// this pin i snot correct it is used for alerting the operator that the battery is less than 3.2v
// Charging Management Function with Pre-Measured Values
void chargingManagement(float battery_voltage, float temperature) {
    while (isCharging()) {  // Loop as long as the robot is in charging mode
        static unsigned long previousMillis = 0;
        const unsigned long interval = 500;  // Check conditions every 500ms
        unsigned long currentMillis = millis();

        // Perform condition checks only at specified intervals
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;

            battery_voltage = monitorBattery();
            temperature = monitorTemperature();

            // Check if both conditions for safe charging are met
            if (battery_voltage < MAX_VOLTAGE && temperature < HIGH_TEMP_THRESHOLD) {
                Serial.println("Both conditions met: Starting charging...");
                digitalWrite(RELAY_PIN,LOW);  // Start charging
                updateLEDStatus(battery_voltage, true, false);  // Update LED to indicate charging
            }
            // If combined conditions are not met, check individual conditions
            else {
                if (battery_voltage >= MAX_VOLTAGE) {
                    Serial.println("Battery voltage is too high: Stopping charging...");
                    digitalWrite(RELAY_PIN, HIGH);  // Stop charging
                    updateLEDStatus(battery_voltage, true, false);  // Update LED to show high voltage
                }

                if (temperature >= HIGH_TEMP_THRESHOLD) {
                    Serial.println("Temperature is too high: Stopping charging...");
                    digitalWrite(RELAY_PIN, HIGH);  // Stop charging
                    updateLEDStatus(battery_voltage, true, true);  // Update LED to show high temperature
                    Serial.println("Waiting for temperature to cool down...");

                    // Wait until temperature falls below the low threshold
                    while (monitorTemperature() > LOW_TEMP_THRESHOLD) {
                        updateLEDStatus(battery_voltage, false, true);  // Continuously update LED
                        delay(500);  // Optional: slow down checks
                    }

                    Serial.println("Temperature cooled down: Restarting checks...");
                    continue;  // Restart the loop to recheck all conditions
                }
            }
        }
    }
}



// Function to stop non-essential components
void stopFunctions() {
    Serial.println("Stopping non-essential components...");
    digitalWrite(POWERBOOST_ENABLE, LOW); // Disable PowerBoost
    digitalWrite(MOTORS_SLEEP, LOW);      // Put motors to sleep
}

// Function to enable non-essential components
void enableFunctions() {
    Serial.println("Enabling non-essential components...");
    digitalWrite(POWERBOOST_ENABLE, HIGH); // Enable PowerBoost
    digitalWrite(MOTORS_SLEEP, HIGH);      // Wake up motors
}






