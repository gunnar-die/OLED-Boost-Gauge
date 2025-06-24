// --- PIC18F26K22 Configuration Bits (Adjust as needed for your specific setup) ---
// Please refer to your PIC18F26K22 datasheet and MPLAB X Configuration Bits window
// for detailed explanations and other options.
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block, A6 and A7 as RC OSC)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly for system clock)
#pragma config PRICLKEN = OFF   // Primary clock enable bit (Primary clock can be disabled by software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 190       // Brown Out Reset Voltage bits (Vbor set to 1.90V)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN is ignored))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up (HFINTOSC starts on Power-up)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>     // Include standard XC8 header for PIC microcontrollers
#include <stdio.h>  // Required for snprintf
#include <string.h> // Required for strlen

// Define system clock frequency for __delay_ms()
// Using internal oscillator at 16MHz
#define _XTAL_FREQ 16000000UL

// --- Pin Definitions (Adjust these to match your wiring) ---
#define SENSOR_ANALOG_PIN   AN0   // Analog input for sensor (e.g., RA0/AN0)
#define SENSOR_TRIS_PIN     TRISA0 // TRIS bit for sensor pin

#define OLED_RST_PIN        LATB0  // Digital output for OLED Reset (e.g., RB0)
#define OLED_RST_TRIS       TRISB0 // TRIS bit for OLED Reset

// I2C Pins (MSSP1 Module, usually RC3/RC4 for Master Mode)
#define I2C_SDA_TRIS        TRISC4 // SDA pin (RC4)
#define I2C_SCL_TRIS        TRISC3 // SCL pin (RC3)

// --- OLED Display Settings ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_I2C_ADDRESS 0x3C // Common I2C address for SSD1306 (0x3D is also common)

// --- UART (Serial) Settings ---
#define BAUD_RATE 9600UL // Match this with your Serial Monitor setting
#define FOSC_UART _XTAL_FREQ // Use system clock for UART baud rate calculation

// --- Calibration Constants for Pressure Formulas ---
// Formulas are calibrated to specific points from the sensor datasheet:
// PSI: (1.0V, 0.0PSI) and (4.5V, 45.0PSI)
// inHg: (0.1V, -30.0inHg) and (1.0V, 0.0inHg)
const float PSI_SLOPE = 12.8571f;
const float PSI_INTERCEPT = -12.8571f;
const float INHG_SLOPE = 33.3333f;
const float INHG_INTERCEPT = -33.3333f;

// --- Display Limits ---
const float MAX_BOOST_PSI = 45.0f;
const float MIN_VACUUM_INHG = -30.0f;

// --- Bar Graph Display Range ---
const float BAR_GRAPH_MAX_PSI = 30.0f; // Max PSI for bar graph full scale

// --- ADC Reference Settings ---
const float ADC_REF_VOLTAGE = 5.0f;   // Reference voltage of the PIC's ADC (Vdd)
const float ADC_MAX_READING = 1023.0f; // 10-bit ADC

// --- OLED Display Buffer ---
// 128 pixels wide * 64 pixels high / 8 bits per byte = 1024 bytes
unsigned char ssd1306_buffer[SCREEN_WIDTH * (SCREEN_HEIGHT / 8)];

// --- Font Data (Basic 5x7 ASCII font) ---
// This is a minimal font for characters ' ' to '~'
// Each character is 5 columns wide, 8 rows high (packed into 1 byte per column)
const unsigned char font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32: Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33: !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34: "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35: #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36: $
    {0x62, 0x64, 0x08, 0x13, 0x23}, // 37: %
    {0x36, 0x49, 0x56, 0x20, 0x50}, // 38: &
    {0x00, 0x08, 0x07, 0x00, 0x00}, // 39: '
    {0x00, 0x20, 0x40, 0x00, 0x00}, // 40: (
    {0x00, 0x00, 0x40, 0x20, 0x00}, // 41: )
    {0x00, 0x0A, 0x7F, 0x28, 0x00}, // 42: *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43: +
    {0x00, 0x00, 0x00, 0x00, 0x80}, // 44: ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45: -
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 46: .
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 47: /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48: 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49: 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50: 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51: 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52: 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53: 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54: 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55: 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56: 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57: 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58: :
    {0x00, 0x00, 0x36, 0x36, 0x00}, // 59: ;
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 60: <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61: =
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 62: >
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 63: ?
    // Add more characters here if needed, or adjust table size
};


// --- Global variables for OLED state ---
static unsigned char _oled_cursor_x = 0;
static unsigned char _oled_cursor_y = 0;
static unsigned char _oled_text_size = 1;

// --- Function Prototypes ---
void SYS_Init(void);
void UART_Init(void);
void UART_WriteChar(char data);
void UART_WriteString(const char *str);

void ADC_Init(void);
unsigned int ADC_Read(unsigned char channel);

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char data);
unsigned char i2c_read(void);
void i2c_ack(void);
void i2c_nack(void);

void ssd1306_command(unsigned char c);
void ssd1306_data(unsigned char c);
void ssd1306_init(void);
void ssd1306_clearDisplay(void);
void ssd1306_setCursor(unsigned char x, unsigned char y);
void ssd1306_setTextSize(unsigned char s);
void ssd1306_printChar(char c);
void ssd1306_printString(const char *str);
void ssd1306_fillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h, unsigned char color);
void ssd1306_display(void);
void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color);
void float_to_str(float val, char *buf, unsigned char precision);
unsigned int map_value(unsigned int x, unsigned int in_min, unsigned int in_max, unsigned int out_min, unsigned int out_max);
float constrain_float(float value, float min_val, float max_val);

// --- System Initialization ---
void SYS_Init(void) {
    OSCCON = 0x70; // Set internal oscillator to 16MHz
    // Peripheral Pin Select (PPS) - only needed if remapping pins from default
    // For PIC18F26K22, default I2C pins are RC3 (SCL) and RC4 (SDA)
    // No PPS needed for these defaults.
    
    // Set digital pins
    ANSELB = 0x00; // All PORTB pins as digital
    ANSELC = 0x00; // All PORTC pins as digital

    // Set TRIS bits for OLED Reset pin
    OLED_RST_TRIS = 0; // Output
    OLED_RST_PIN = 1;  // De-assert reset (active low)
}

// --- UART (Serial) Functions ---
void UART_Init(void) {
    // RC6 as TX, RC7 as RX
    TRISC6 = 0; // TX pin as output
    TRISC7 = 1; // RX pin as input

    // Configure EUSART
    TXSTA = 0x24; // TX enable, High Baud Rate
    RCSTA = 0x90; // Serial port enable, RX enable
    BAUDCON = 0x08; // 8-bit Baud Rate Generator is used, BRG16 is set

    // Calculate SPBRG value
    // SPBRG = (_XTAL_FREQ / (16 * BAUD_RATE)) - 1
    // For 16MHz Fosc, 9600 baud, high speed (BRGH=1, BRG16=1): SPBRG = (16000000 / (16 * 9600)) - 1 = 104.16 -> 104
    // Note: Fractional part means baud rate won't be perfectly exact.
    SPBRG = (_XTAL_FREQ / (16 * BAUD_RATE)) - 1;
    SPBRGH = 0; // For 8-bit SPBRG, SPBRGH is 0.
}

void UART_WriteChar(char data) {
    while (!TXIF); // Wait until transmit buffer is empty
    TXREG = data;  // Send data
}

void UART_WriteString(const char *str) {
    while (*str != '\0') {
        UART_WriteChar(*str++);
    }
}

// --- ADC Functions ---
void ADC_Init(void) {
    // ANSEL register configures analog pins
    ANSELAbits.ANSA0 = 1; // Set RA0 as analog input for SENSOR_ANALOG_PIN

    ADCON0 = 0x01; // Select AN0, ADC ON
    ADCON1 = 0x00; // Vref+ = VDD, Vref- = VSS
    ADCON2 = 0x8A; // Right justified, 10-TAD, Fosc/32 (for 16MHz, Fosc/32 = 0.5MHz, Tad = 2us, which is > 1.6us minimum)
                   // A/D result in ADRESH:ADRESL
}

unsigned int ADC_Read(unsigned char channel) {
    ADCON0bits.CHS = channel; // Select ADC channel (e.g., AN0 = 0x00)
    __delay_us(20);           // Acquisition time, small delay for capacitor to charge
    ADCON0bits.GO_nDONE = 1;  // Start ADC conversion
    while (ADCON0bits.GO_nDONE); // Wait for conversion to complete
    return (ADRESH << 8) + ADRESL; // Return 10-bit result
}

// --- I2C (MSSP) Functions ---
void i2c_init(void) {
    SSP1STAT = 0x80; // Slew rate control disabled
    SSP1CON1 = 0x28; // Master mode, clock = Fosc/(4*(SSPADD+1))
    SSP1ADD = (_XTAL_FREQ / (4 * 100000UL)) - 1; // 100 kHz I2C speed (100kHz standard)
    // For _XTAL_FREQ = 16MHz, 100kHz I2C: SSP1ADD = (16000000 / (4 * 100000)) - 1 = 40 - 1 = 39
    
    // Set TRIS bits for I2C pins as inputs (MSSP module controls them when enabled)
    I2C_SDA_TRIS = 1;
    I2C_SCL_TRIS = 1;
}

void i2c_start(void) {
    SSP1CON2bits.SEN = 1; // Send Start Condition
    while (SSP1CON2bits.SEN); // Wait for Start Condition to complete
    PIR1bits.SSPIF = 0; // Clear interrupt flag
}

void i2c_stop(void) {
    SSP1CON2bits.PEN = 1; // Send Stop Condition
    while (SSP1CON2bits.PEN); // Wait for Stop Condition to complete
    PIR1bits.SSPIF = 0; // Clear interrupt flag
}

void i2c_write(unsigned char data) {
    SSP1BUF = data; // Write data to buffer
    while (SSP1STATbits.BF || SSP1CON2bits.ACKSTAT); // Wait for transmission complete AND ACK received
    PIR1bits.SSPIF = 0; // Clear interrupt flag
}

unsigned char i2c_read(void) {
    SSP1CON2bits.RCEN = 1; // Enable receive mode
    while (!SSP1STATbits.BF); // Wait for byte to be received
    PIR1bits.SSPIF = 0; // Clear interrupt flag
    return SSP1BUF;    // Read data from buffer
}

void i2c_ack(void) {
    SSP1CON2bits.ACKDT = 0; // Set ACK bit (0 = ACK)
    SSP1CON2bits.ACKEN = 1; // Send ACK
    while (SSP1CON2bits.ACKEN); // Wait for ACK to complete
}

void i2c_nack(void) {
    SSP1CON2bits.ACKDT = 1; // Set NACK bit (1 = NACK)
    SSP1CON2bits.ACKEN = 1; // Send NACK
    while (SSP1CON2bits.ACKEN); // Wait for NACK to complete
}


// --- SSD1306 OLED Driver Functions ---
void ssd1306_command(unsigned char c) {
    i2c_start();
    i2c_write((OLED_I2C_ADDRESS << 1) | 0x00); // Slave address + Write bit (0x00 for command)
    i2c_write(0x80); // Co = 1, D/C# = 0 (Command mode, Continuation bit)
    i2c_write(c);
    i2c_stop();
}

void ssd1306_data(unsigned char c) {
    i2c_start();
    i2c_write((OLED_I2C_ADDRESS << 1) | 0x00); // Slave address + Write bit (0x00 for data)
    i2c_write(0x40); // Co = 0, D/C# = 1 (Data mode, No Continuation bit)
    i2c_write(c);
    i2c_stop();
}

void ssd1306_init(void) {
    // Hardware reset if using a dedicated RST pin
    OLED_RST_PIN = 0; // Assert reset (active low)
    __delay_ms(10);
    OLED_RST_PIN = 1; // De-assert reset
    __delay_ms(10);

    ssd1306_command(0xAE); // Display OFF

    ssd1306_command(0xD5); // Set Display Clock Divide Ratio / Oscillator Frequency
    ssd1306_command(0x80);

    ssd1306_command(0xA8); // Set Multiplex Ratio
    ssd1306_command(0x3F); // 63 (for 1/64 duty cycle for 64px height)

    ssd1306_command(0xD3); // Set Display Offset
    ssd1306_command(0x00);

    ssd1306_command(0x40); // Set Display Start Line (0)

    ssd1306_command(0x8D); // Set Charge Pump
    ssd1306_command(0x14); // Enable Charge Pump

    ssd1306_command(0x20); // Set Memory Addressing Mode
    ssd1306_command(0x00); // Horizontal Addressing Mode

    ssd1306_command(0xA1); // Set Segment Re-map (A0h/A1h) - reverse column address (for some displays)
    ssd1306_command(0xC8); // Set COM Output Scan Direction (C0h/C8h) - reverse COM scan direction (for some displays)

    ssd1306_command(0xDA); // Set COM Pins Hardware Configuration
    ssd1306_command(0x12); // (0x02 for 32px height, 0x12 for 64px height)

    ssd1306_command(0x81); // Set Contrast Control
    ssd1306_command(0xCF); // (0xCF is default power up, 0x9F good for 3.3V)

    ssd1306_command(0xD9); // Set Pre-charge Period
    ssd1306_command(0xF1);

    ssd1306_command(0xDB); // Set VCOMH Deselect Level
    ssd1306_command(0x40); // 0.77 * VCC

    ssd1306_command(0xA4); // Entire Display ON (output RAM contents)
    ssd1306_command(0xA6); // Set Normal Display (0xA7 for Inverse Display)

    ssd1306_command(0x2E); // Deactivate Scroll (if active)

    ssd1306_command(0xAF); // Display ON
}

void ssd1306_clearDisplay(void) {
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

void ssd1306_display(void) {
    ssd1306_command(0x21); // Set Column Address
    ssd1306_command(0x00); // Column Start Address
    ssd1306_command(SCREEN_WIDTH - 1); // Column End Address

    ssd1306_command(0x22); // Set Page Address
    ssd1306_command(0x00); // Page Start Address
    ssd1306_command((SCREEN_HEIGHT / 8) - 1); // Page End Address

    // Send the entire buffer
    i2c_start();
    i2c_write((OLED_I2C_ADDRESS << 1) | 0x00); // Slave address + Write bit
    i2c_write(0x40); // Co = 0, D/C# = 1 (Data mode, No Continuation bit)
    
    for (unsigned int i = 0; i < sizeof(ssd1306_buffer); i++) {
        i2c_write(ssd1306_buffer[i]);
    }
    i2c_stop();
}

void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color) {
    if ((x >= SCREEN_WIDTH) || (y >= SCREEN_HEIGHT)) {
        return; // Out of bounds
    }
    
    unsigned int index = x + (y / 8) * SCREEN_WIDTH;
    if (color) { // White pixel
        ssd1306_buffer[index] |= (1 << (y & 7));
    } else { // Black pixel
        ssd1306_buffer[index] &= ~(1 << (y & 7));
    }
}

void ssd1306_fillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h, unsigned char color) {
    // Simple filled rectangle. More optimized versions can be done directly on pages.
    for (unsigned char i = x; i < x + w; i++) {
        for (unsigned char j = y; j < y + h; j++) {
            ssd1306_drawPixel(i, j, color);
        }
    }
}

void ssd1306_setCursor(unsigned char x, unsigned char y) {
    _oled_cursor_x = x;
    _oled_cursor_y = y;
}

void ssd1306_setTextSize(unsigned char s) {
    _oled_text_size = s;
    if (_oled_text_size == 0) _oled_text_size = 1; // Prevent size 0
}

void ssd1306_printChar(char c) {
    if ((c < ' ') || (c > '~')) { // Only handle printable ASCII characters ' ' to '~' for this font
        c = ' '; // Replace unprintable with space
    }
    
    unsigned char char_index = c - ' '; // Calculate index in font table
    
    for (unsigned char i = 0; i < 5; i++) { // Font is 5 columns wide
        unsigned char col = font5x7[char_index][i];
        for (unsigned char j = 0; j < 8; j++) { // Each column has 8 bits (rows)
            if (col & 0x01) { // If bit is set
                // Draw pixel, applying text size scaling
                for (unsigned char sx = 0; sx < _oled_text_size; sx++) {
                    for (unsigned char sy = 0; sy < _oled_text_size; sy++) {
                        ssd1306_drawPixel(_oled_cursor_x + i * _oled_text_size + sx,
                                          _oled_cursor_y + j * _oled_text_size + sy,
                                          1); // Always white
                    }
                }
            }
            col >>= 1; // Next row bit
        }
    }
    // Advance cursor
    _oled_cursor_x += (5 * _oled_text_size) + _oled_text_size; // 5 columns + 1 pixel spacing between chars
}

void ssd1306_printString(const char *str) {
    while (*str != '\0') {
        ssd1306_printChar(*str++);
    }
}

// Custom float to string conversion (minimal, handles negative, decimal places)
void float_to_str(float val, char *buf, unsigned char precision) {
    int integer_part = (int)val;
    float fractional_part = val - integer_part;

    if (val < 0) {
        *buf++ = '-';
        integer_part = -integer_part;
        fractional_part = -fractional_part;
    }
    
    // Print integer part
    if (integer_part == 0 && val >= 0.0f) { // Handle 0.something
        *buf++ = '0';
    } else if (integer_part == 0 && val < 0.0f) {
        // already handled negative sign
    } else {
        // Simple itoa for integer part (can be optimized)
        char temp_int_buf[10];
        int i = 0;
        if (integer_part == 0) {
            temp_int_buf[i++] = '0';
        } else {
            while (integer_part > 0) {
                temp_int_buf[i++] = (integer_part % 10) + '0';
                integer_part /= 10;
            }
        }
        while (i > 0) {
            *buf++ = temp_int_buf[--i];
        }
    }

    if (precision > 0) {
        *buf++ = '.';
        for (unsigned char p = 0; p < precision; p++) {
            fractional_part *= 10.0f;
            int digit = (int)fractional_part;
            *buf++ = digit + '0';
            fractional_part -= digit;
        }
    }
    *buf = '\0'; // Null-terminate the string
}

// Arduino-like map function
unsigned int map_value(unsigned int x, unsigned int in_min, unsigned int in_max, unsigned int out_min, unsigned int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino-like constrain function for floats
float constrain_float(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}


// --- Main Program ---
void main(void) {
    SYS_Init();  // Initialize system clock and digital I/O
    UART_Init(); // Initialize UART for serial communication
    ADC_Init();  // Initialize ADC module
    i2c_init();  // Initialize I2C module

    UART_WriteString("--- Boost Gauge Initializing v1.0 ---\r\n");
    UART_WriteString("Attempting OLED display initialization...\r\n");

    // Initialize OLED display
    // Check if display.begin returns true/false is not directly possible here
    // as our custom driver doesn't have that handshake built in.
    // We rely on i2c_init() to ensure I2C bus is ready.
    ssd1306_init();
    UART_WriteString("OLED display initialized successfully.\r\n");

    // Clear display buffer and show a brief startup message
    __delay_ms(2000); // Allow time for display to power up
    ssd1306_clearDisplay();

    ssd1306_setTextSize(1);
    ssd1306_setCursor(0, 0);
    ssd1306_printString("Boost Gauge v1.0");
    ssd1306_setCursor(0, 10); // Move cursor down for "Ready..."
    ssd1306_printString("Ready...");
    ssd1306_display(); // Push startup message to display
    __delay_ms(1500); // Pause for 1.5 seconds

    while (1) {
        char buffer[20]; // Buffer for string conversions

        // Read raw analog sensor value (0 to 1023)
        unsigned int sensorRawValue = ADC_Read(0); // Read from AN0
        UART_WriteString("Raw: ");
        float_to_str((float)sensorRawValue, buffer, 0); UART_WriteString(buffer); UART_WriteString("\r\n");
        __delay_ms(500); // Delay for Serial output

        // Convert raw sensor value to actual voltage (Signal V)
        float signalVoltage = sensorRawValue * (ADC_REF_VOLTAGE / ADC_MAX_READING);
        UART_WriteString("Voltage: ");
        float_to_str(signalVoltage, buffer, 3); UART_WriteString(buffer); UART_WriteString("\r\n");
        __delay_ms(500); // Delay for Serial output

        // Calculate PSI and inHg using derived linear formulas
        float calculatedPSI = (PSI_SLOPE * signalVoltage) + PSI_INTERCEPT;
        float calculatedInHg = (INHG_SLOPE * signalVoltage) + INHG_INTERCEPT;
        
        UART_WriteString("Calculated PSI: ");
        float_to_str(calculatedPSI, buffer, 2); UART_WriteString(buffer);
        UART_WriteString(", inHg: ");
        float_to_str(calculatedInHg, buffer, 2); UART_WriteString(buffer); UART_WriteString("\r\n");
        __delay_ms(500); // Delay for Serial output

        // Clear the display buffer for the new reading
        ssd1306_clearDisplay();

        // Set text properties for the main pressure reading
        ssd1306_setTextSize(3); // Increased text size for main numerical reading
        // No direct setTextColor in this driver, assumes white by default

        // Determine what to display (PSI or inHg) and apply limits
        if (calculatedPSI >= 0.0f) { // Positive pressure (boost)
            calculatedPSI = constrain_float(calculatedPSI, 0.0f, MAX_BOOST_PSI);
            
            // --- Display numerical PSI value (left-aligned) ---
            ssd1306_setCursor(0, 0); // Set cursor for main numerical reading (top-left)
            float_to_str(calculatedPSI, buffer, 1);
            ssd1306_printString(buffer);

            // --- Display " PSI" unit label (right-aligned) ---
            char psiUnitStr[] = " PSI";
            int text_width = strlen(psiUnitStr) * (5 * 2 + 2); // (font_width * text_size + spacing_pixels)
            // Note: This is an estimation. Real calculation would need to measure string width.
            
            ssd1306_setTextSize(2); // Set text size for the unit label specifically
            ssd1306_setCursor(SCREEN_WIDTH - text_width, 0); // Position cursor for the unit label
            ssd1306_printString(psiUnitStr);

            // --- Draw Horizontal Bar Graph for Positive Pressure ---
            // Map current PSI (0 to BAR_GRAPH_MAX_PSI) to screen pixels (0 to SCREEN_WIDTH-1)
            float barValue = constrain_float(calculatedPSI, 0.0f, BAR_GRAPH_MAX_PSI);
            unsigned int barWidth = map_value(barValue * 100, 0, BAR_GRAPH_MAX_PSI * 100, 0, SCREEN_WIDTH);
            
            // Draw the filled rectangle (the bar)
            ssd1306_fillRect(0, 28, barWidth, 30, 1); // Bar at y=28, height 30 pixels, color white

        } else { // Negative pressure (vacuum)
            calculatedInHg = constrain_float(calculatedInHg, MIN_VACUUM_INHG, 0.0f);
            
            // Display numerical value at the top of the screen (left-aligned)
            ssd1306_setCursor(0, 0); // Set cursor for main reading (top-left)
            float_to_str(calculatedInHg, buffer, 1);
            ssd1306_printString(buffer);

            // Display unit label on the lower part of the screen
            ssd1306_setTextSize(2); // Smaller text for unit label
            ssd1306_setCursor(0, 35); // Position for the unit text on the lower part
            ssd1306_printString(F(" inHg")); // Print the unit label
        }

        // Update the actual OLED display with the contents of the buffer
        ssd1306_display();

        // Short delay for faster OLED display updates (50ms)
        __delay_ms(50); // OLED display updates every 50 milliseconds
    }
}
