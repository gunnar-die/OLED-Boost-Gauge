#include <Wire.h>          // Required for I2C communication with the OLED
#include <Adafruit_GFX.h>  // Core graphics library
#include <Adafruit_SSD1306.h> // Library for SSD1306 OLED displays

// --- OLED Display Settings ---
#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET 4 // Reset pin # (can be any digital pin, or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< Common I2C address for 128x64 SSD1306 OLEDs

// Create the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Sensor Settings ---
const int SENSOR_PIN = A0; // Analog input pin for your pressure sensor

// --- Calibration Constants for Pressure Formulas ---
// PSI = (PSI_SLOPE * Signal V) + PSI_INTERCEPT
const float PSI_SLOPE = 15.5128;
const float PSI_INTERCEPT = -16.0692;

// Linear correction constants for PSI display output (Actual_PSI = M * OLED_PSI_Reading + B)
const float PSI_CORRECTION_M = 0.86478;
const float PSI_CORRECTION_B = -2.76730;

// inHg = (INHG_SLOPE * Signal V) + INHG_INTERCEPT
const float INHG_SLOPE = 33.3333;
const float INHG_INTERCEPT = -33.3333;

// --- Display Limits ---
const float MAX_BOOST_PSI = 45.0;
const float MIN_VACUUM_INHG = -30.0;

// --- Bar Graph Display Range ---
const float BAR_GRAPH_MAX_PSI = 30.0;
const float BAR_GRAPH_MAX_INHG = 30.0;

// --- ADC (Analog-to-Digital Converter) Reference Settings ---
const float ADC_REF_VOLTAGE = 5.0;
const float ADC_MAX_READING = 1023.0;

// --- Timing variables for non-blocking Serial output ---
unsigned long lastSerialPrintTime = 0;
const long serialPrintInterval = 1500; // Print to serial every 1.5 seconds

void setup() {
  Serial.begin(9600);
  Serial.println(F("--- Boost Gauge Initializing v1.0 ---"));
  Serial.println(F("Attempting OLED display initialization..."));

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("ERROR: SSD1306 allocation failed. Check wiring, address (0x3C or 0x3D?), and reset pin."));
    for (;;); // Halt if display initialization fails
  }
  Serial.println(F("OLED display initialized successfully."));

  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Boost Gauge v1.0"));
  display.println(F("Ready..."));
  display.display();
  delay(1500);
}

void loop() {
  int sensorRawValue = analogRead(SENSOR_PIN);
  float signalVoltage = sensorRawValue * (ADC_REF_VOLTAGE / ADC_MAX_READING);
  float calculatedPSI = (PSI_SLOPE * signalVoltage) + PSI_INTERCEPT;
  float calculatedInHg = (INHG_SLOPE * signalVoltage) + INHG_INTERCEPT;

  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialPrintTime >= serialPrintInterval) {
    lastSerialPrintTime = currentMillis;

    Serial.print(F("Raw: "));
    Serial.println(sensorRawValue);
    
    Serial.print(F("Voltage: "));
    Serial.println(signalVoltage, 3);
    
    Serial.print(F("Calculated PSI: "));
    Serial.print(calculatedPSI, 2);
    Serial.print(F(", inHg: "));
    Serial.println(calculatedInHg, 2);
  }

  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);

  if (calculatedPSI >= 0.0) { // Positive pressure (boost)
    calculatedPSI = (PSI_CORRECTION_M * calculatedPSI) + PSI_CORRECTION_B;
    if (calculatedPSI > MAX_BOOST_PSI) {
      calculatedPSI = MAX_BOOST_PSI;
    }
    
    display.setCursor(0, 0);
    display.print(calculatedPSI, 1);

    char psiUnitStr[] = " PSI";
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextSize(2);
    display.getTextBounds(psiUnitStr, 0, 0, &x1, &y1, &w, &h);
    int psiUnitX = SCREEN_WIDTH - w;
    display.setCursor(psiUnitX, 0);
    display.print(F(" PSI"));

    int barWidth = map(calculatedPSI * 100, 0 * 100, BAR_GRAPH_MAX_PSI * 100, 0, SCREEN_WIDTH);
    barWidth = constrain(barWidth, 0, SCREEN_WIDTH);
    display.fillRect(0, 28, barWidth, 30, SSD1306_WHITE);

    int barBaseY_psi = 28;
    int barHeight_psi = 30;
    int y_center_psi = barBaseY_psi + barHeight_psi / 2;
    int y_top_psi = barBaseY_psi;
    int y_bottom_psi = barBaseY_psi + barHeight_psi;
    int arm_length = 10;

    for (int x_draw = 0; x_draw < barWidth; x_draw += 10) {
        for (int i = 0; i < 4; i++) {
            display.drawLine(x_draw + i, y_top_psi,
                             x_draw + arm_length + i, y_center_psi,
                             SSD1306_BLACK);
            display.drawLine(x_draw + i, y_bottom_psi,
                             x_draw + arm_length + i, y_center_psi,
                             SSD1306_BLACK);
        }
    }

  } else { // Negative pressure (vacuum)
    if (calculatedInHg < MIN_VACUUM_INHG) {
      calculatedInHg = MIN_VACUUM_INHG;
    }
    display.setCursor(0, 0);
    display.print(calculatedInHg, 1);

    char inHgUnitStr[] = " inHg";
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextSize(1);
    display.getTextBounds(inHgUnitStr, 0, 0, &x1, &y1, &w, &h);
    int inHgUnitX = SCREEN_WIDTH - w;
    display.setCursor(inHgUnitX, 0);
    display.print(F(" inHg"));

    float inHgNormalized = abs(calculatedInHg);
    int barWidth = map(inHgNormalized * 100, 0 * 100, BAR_GRAPH_MAX_INHG * 100, 0, SCREEN_WIDTH);
    barWidth = constrain(barWidth, 0, SCREEN_WIDTH);

    int barBaseY_inHg = 40;
    int barHeight_inHg = 10;

    display.fillRect(SCREEN_WIDTH - barWidth, barBaseY_inHg, barWidth, barHeight_inHg, SSD1306_WHITE);

    for (int x_slash_start = SCREEN_WIDTH - barWidth; x_slash_start < SCREEN_WIDTH; x_slash_start += 5) {
        display.drawLine(x_slash_start, barBaseY_inHg,
                         x_slash_start + barHeight_inHg - 1, barBaseY_inHg + barHeight_inHg - 1,
                         SSD1306_BLACK);
        display.drawLine(x_slash_start + 1, barBaseY_inHg,
                         x_slash_start + barHeight_inHg, barBaseY_inHg + barHeight_inHg - 1,
                         SSD1306_BLACK);
    }
  }

  display.display();
  delay(50);
}
