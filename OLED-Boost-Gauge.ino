#include <Wire.h>          // Required for I2C communication with the OLED
#include <Adafruit_GFX.h>  // Core graphics library
#include <Adafruit_SSD1306.h> // Library for SSD1306 OLED displays

// --- OLED Display Settings ---
#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins typically A4 and A5 on Nano)
#define OLED_RESET 4 // Reset pin # (can be any digital pin, or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< Common I2C address for 128x64 SSD1306 OLEDs (0x3D is also common, check your module)

// Create the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Sensor Settings ---
const int SENSOR_PIN = A0; // Analog input pin for your pressure sensor

// --- Calibration Constants for Pressure Formulas ---
// Formulas are calibrated to specific points from the sensor datasheet:
// PSI: (1.0V, 0.0PSI) and (4.5V, 45.0PSI)
// inHg: (0.1V, -30.0inHg) and (1.0V, 0.0inHg)

// PSI = (PSI_SLOPE * Signal V) + PSI_INTERCEPT
const float PSI_SLOPE = 12.8571;
const float PSI_INTERCEPT = -12.8571;

// inHg = (INHG_SLOPE * Signal V) + INHG_INTERCEPT
const float INHG_SLOPE = 33.3333;
const float INHG_INTERCEPT = -33.3333;

// --- Display Limits ---
const float MAX_BOOST_PSI = 45.0;    // Maximum boost pressure to display in PSI (overall sensor range)
const float MIN_VACUUM_INHG = -30.0; // Minimum vacuum pressure to display in inHg (this value is negative)

// --- Bar Graph Display Range ---
const float BAR_GRAPH_MAX_PSI = 30.0; // Max PSI for bar graph full scale (0 to SCREEN_WIDTH)

// --- ADC (Analog-to-Digital Converter) Reference Settings ---
// Standard Arduino ADC uses a 5V reference voltage and has 10-bit resolution (0-1023)
const float ADC_REF_VOLTAGE = 5.0;   // Reference voltage of the Arduino's ADC (usually 5V for Nano)
const float ADC_MAX_READING = 1023.0; // Maximum raw value from analogRead()

void setup() {
  // Initialize serial communication for debugging. Open Serial Monitor at 9600 baud.
  Serial.begin(9600);
  Serial.println(F("--- Boost Gauge Initializing ---"));
  Serial.println(F("Attempting OLED display initialization..."));

  // Initialize OLED display. SSD1306_SWITCHCAPVCC generates display voltage internally.
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("ERROR: SSD1306 allocation failed. Check wiring, address (0x3C or 0x3D?), and reset pin."));
    for (;;); // Halt if display initialization fails
  }
  Serial.println(F("OLED display initialized successfully."));

  // Clear display buffer and show a brief startup message
  delay(2000); // Allow time for display to power up
  display.clearDisplay();

  display.setTextSize(1);       // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);      // Start at the top-left corner
  display.println(F("Boost Gauge"));
  display.println(F("Ready..."));
  display.display(); // Push startup message to display
  delay(1500);       // Pause for 1.5 seconds
}

void loop() {
  // Read raw analog sensor value (0 to 1023)
  int sensorRawValue = analogRead(SENSOR_PIN);
  Serial.print(F("Raw: "));
  Serial.println(sensorRawValue);
  delay(500); // Delay for Serial output

  // Convert raw sensor value to actual voltage (Signal V)
  float signalVoltage = sensorRawValue * (ADC_REF_VOLTAGE / ADC_MAX_READING);
  Serial.print(F("Voltage: "));
  Serial.println(signalVoltage, 3);
  delay(500); // Delay for Serial output

  // Calculate PSI and inHg using derived linear formulas
  float calculatedPSI = (PSI_SLOPE * signalVoltage) + PSI_INTERCEPT;
  float calculatedInHg = (INHG_SLOPE * signalVoltage) + INHG_INTERCEPT;
  Serial.print(F("Calculated PSI: "));
  Serial.print(calculatedPSI, 2);
  Serial.print(F(", inHg: "));
  Serial.println(calculatedInHg, 2);
  delay(500); // Delay for Serial output

  // Clear the display buffer for the new reading
  display.clearDisplay();

  // Set text properties for the main pressure reading
  display.setTextSize(3); // Increased text size for main numerical reading
  display.setTextColor(SSD1306_WHITE);

  // Determine what to display (PSI or inHg) and apply limits
  if (calculatedPSI >= 0.0) { // Positive pressure (boost)
    if (calculatedPSI > MAX_BOOST_PSI) {
      calculatedPSI = MAX_BOOST_PSI; // Clamp to max PSI (for numerical display if it goes over sensor's max)
    }
    
    // --- Display numerical PSI value (left-aligned) ---
    display.setCursor(0, 0); // Set cursor for main numerical reading (top-left)
    display.print(calculatedPSI, 1); // Print PSI with 1 decimal place

    // --- Display " PSI" unit label (right-aligned) ---
    char psiUnitStr[] = " PSI";
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextSize(2); // Set text size for the unit label specifically
    display.getTextBounds(psiUnitStr, 0, 0, &x1, &y1, &w, &h); // Get width of " PSI"
    
    // Calculate X position to right-align " PSI"
    int psiUnitX = SCREEN_WIDTH - w;
    display.setCursor(psiUnitX, 0); // Position cursor for the unit label
    display.print(F(" PSI")); // Print the unit label

    // --- Draw Horizontal Bar Graph for Positive Pressure ---
    // Map current PSI (0 to BAR_GRAPH_MAX_PSI) to screen pixels (0 to SCREEN_WIDTH-1)
    // Values above BAR_GRAPH_MAX_PSI will result in full bar due to `constrain`
    int barWidth = map(calculatedPSI * 100, 0 * 100, BAR_GRAPH_MAX_PSI * 100, 0, SCREEN_WIDTH);
    barWidth = constrain(barWidth, 0, SCREEN_WIDTH); // Ensure barWidth stays within display bounds

    // Draw the filled rectangle (the bar)
    display.fillRect(0, 28, barWidth, 30, SSD1306_WHITE); // Bar at y=28, height 30 pixels

  } else { // Negative pressure (vacuum)
    if (calculatedInHg < MIN_VACUUM_INHG) {
      calculatedInHg = MIN_VACUUM_INHG; // Clamp to min inHg
    }
    // Display numerical value at the top of the screen (left-aligned)
    display.setCursor(0, 0); // Set cursor for main reading (top-left)
    display.print(calculatedInHg, 1); // Print inHg with 1 decimal place (it will be negative)

    // Display unit label on the lower part of the screen
    display.setTextSize(2); // Smaller text for unit label
    // Position the unit text below where the bar graph would normally be, vertically centered.
    // The bar graph area starts at y=28 with a height of 30. So, (28 + 30) / 2 = 43 is the center.
    // Text size 2 means 16 pixels high. To center, start at 43 - (16/2) = 43 - 8 = 35.
    display.setCursor(0, 35); // Position for the unit text on the lower part
    display.print(F(" inHg")); // Print the unit label
  }

  // Update the actual OLED display with the contents of the buffer
  display.display();

  // Short delay for faster OLED display updates (50ms)
  delay(50); // OLED display updates every 50 milliseconds
}
