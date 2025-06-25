# Arduino Boost Gauge with OLED Display

This repository contains the Arduino sketch for a custom digital boost and vacuum gauge, designed to provide real-time pressure readings from an analog sensor on an OLED display. It features dynamic PSI (boost) and inHg (vacuum) readouts, visually enhanced with responsive bar graphs.

## Features

  * **Real-time Pressure Monitoring:** Displays current boost pressure in PSI and vacuum pressure in inHg.
  * **Dual Display Modes:** Automatically switches between PSI and inHg based on pressure type (positive or negative).
  * **Dynamic Bar Graphs:**
      * **PSI Bar:** A horizontal bar graph for boost, filling from left to right, featuring a dynamic `>` pattern.
      * **inHg Bar:** A horizontal bar graph for vacuum, filling from right to left, featuring a `\` slash pattern, providing intuitive visual feedback.
  * **OLED Integration:** Utilizes a 128x64 pixel SSD1306 OLED display via I2C for clear readouts.
  * **Adjustable Calibration:** Easily tune the pressure readings to match an external reference gauge for high accuracy.
  * **Non-Blocking Serial Output:** Debugging information is printed to the Serial Monitor at controlled intervals without slowing down the OLED display updates.
  * **Compact Design:** Ideal for vehicle dashboards or custom project enclosures.

## Hardware Components

To build this boost gauge, you will need the following components:

  * **Microcontroller:** Arduino Nano (or compatible ATmega328P based board)
  * **Display:** SSD1306 128x64 OLED Display (I2C interface)
  * **Pressure Sensor:** An analog pressure sensor with a 0-5V output range (e.g., a MAP sensor, or a dedicated boost sensor).
  * **Resistor:** 10kΩ resistor (for pull-down on analog input, if sensor floats when disconnected)
  * **Wiring:** Jumper wires
  * **Power Supply:** 5V DC power supply suitable for your Arduino.

## Wiring Diagram

Connect the components as follows:

**Arduino Nano ↔ SSD1306 OLED (I2C)**

  * `A4 (SDA)` ↔ `SDA`
  * `A5 (SCL)` ↔ `SCL`
  * `D4` ↔ `RST` (OLED Reset pin, define `OLED_RESET` in code)
  * `5V` ↔ `VCC`
  * `GND` ↔ `GND`

**Arduino Nano ↔ Analog Pressure Sensor**

  * `A0` ↔ `Signal Out` (from sensor)
  * `5V` ↔ `VCC` (from sensor)
  * `GND` ↔ `GND` (from sensor)
  * `A0` ↔ `10kΩ Resistor` ↔ `GND` (Optional, but recommended for stable 0V reading when sensor is disconnected)

**Optional: USB-to-Serial Converter (for debugging)**

  * `Arduino TX (D1)` ↔ `RX` (on converter)
  * `Arduino RX (D0)` ↔ `TX` (on converter)
  * `Arduino GND` ↔ `GND` (on converter)

## Software Setup

1.  **Arduino IDE:** Download and install the [Arduino IDE](https://www.google.com/search?q=https://www.arduino.cc/software).
2.  **Libraries:** Install the following libraries via the Arduino IDE's Library Manager (`Sketch > Include Library > Manage Libraries...`):
      * `Adafruit GFX Library`
      * `Adafruit SSD1306`

## Code Structure

The main logic resides within the `loop()` function, which continuously reads the sensor, performs calculations, and updates the OLED. Serial communication is handled asynchronously using `millis()` to avoid slowing down the display.

  * **`setup()`:** Initializes Serial, OLED, and displays a startup message.
  * **`loop()`:**
      * Reads analog voltage from `SENSOR_PIN`.
      * Converts voltage to `calculatedPSI` and `calculatedInHg` using linear formulas.
      * Applies a `PSI_CORRECTION_M` and `PSI_CORRECTION_B` for finer calibration of PSI values.
      * Prints debug data to Serial Monitor at a fixed interval (1.5 seconds).
      * Clears the OLED display.
      * Selects between displaying PSI (positive pressure) or inHg (negative pressure).
      * Draws the corresponding numerical value and unit.
      * Draws a dynamic bar graph with unique patterns for PSI (`>`) and inHg (`\`).
      * Updates the physical OLED display.
      * `delay(50)` ensures a consistent 50ms display refresh rate.

## PIC18F26K22-E Version

An alternative implementation of this boost gauge project is also available using the **Microchip PIC18F26K22-E microcontroller**. This version provides similar functionality but leverages the PIC architecture and its native peripherals.

**Please Note:** The PIC18F26K22-E version is currently **under development** and is provided as a **code skeleton**. It is not yet fully functional or independently runnable without further implementation and debugging of the underlying PIC peripherals (ADC, I2C, UART, and display drawing routines). 
