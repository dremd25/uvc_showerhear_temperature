/**
 * Created: 3/3/2020
 * Author: Darren Remund
 * 
 * Thermistor to tempuature sensor for arduino uno.
 * Thermistor: Vishay BC Components - THERM NTC 2.2KOHM 3520K 0603
 * Resistance to tempurature equation:
 * 
 */


///////////////////////////// Operation Mode /////////////////////////
#define CONVERT_TO_DEG_C      // Uncomment to print tempurature to serial; comment to print resistance instead.
#define LED_WARNING           // Uncomment to light LEDs when tempurature gets too high (uses resistance values).
#define LCD_SCREEN            // Uncomment to use liquid crystal lcd through the scl and sdc pins.

#ifdef LCD_SCREEN
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <stdio.h>
#endif


///////////////////////////// Constant values ////////////////////////
#define BAUD_RATE 9600
#define NUM_THERMISTORS 4
#define INPUT_PIN_1 A0
#define INPUT_PIN_2 A1
#define INPUT_PIN_3 A2
#define INPUT_PIN_4 A3
#define VIN 5
#define R_REF 10000
#define DELAY_MS 250

#ifdef LED_WARNING
  #define WARNING_RES 886.0
  #define ERROR_RES 750.4
  #define OK_PIN_POS 12
  #define OK_PIN_NEG 13
  #define WARNING_PIN_POS 10
  #define WARNING_PIN_NEG 11
  #define ERROR_PIN_POS 8
  #define ERROR_PIN_NEG 9
#endif  // LED_WARNING

#ifdef LCD_SCREEN
#define LCD_ROWS 2
#define LCD_COLUMNS 16
#define LCD_STR_WIDTH (LCD_COLUMNS / 2)
#define LCD_REFRESH_MS 1000
#define LCD_REFRESH_TICKS (LCD_REFRESH_MS / DELAY_MS)
#endif  // LCD_SCREEN

//////////////////////////////// Globals ////////////////////////////
int voltage_raw[NUM_THERMISTORS]; //This is the measured voltage(s)
int pins[NUM_THERMISTORS];

#ifdef LED_WARNING
int temp_ok;
int temp_warning;
int temp_error;
#endif  // LED_WARNING

#ifdef LCD_SCREEN
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);
int lcd_ticks;
bool lcd_print;
char lcd_str[LCD_STR_WIDTH];
#endif  // LCD_SCREEN


/////////////////////////////// Functions //////////////////////////
/**
 * Inline function for converting raw digital value to voltage.
 */
inline float raw_to_resistance(int v) {
  float buffer = v * VIN;     // VIN = max posible voltage (5v) * bit value (0 - 2^10)
  float vout = (buffer)/1024; // Devide value by 2^10 to convert to voltage
  buffer = (VIN / vout) -1;
  float res = R_REF / buffer; // R1 = R2 / (Vin / Vout - 1)

  #ifdef LED_WARNING
  temp_ok = res >= WARNING_RES ? temp_ok : 0;
  temp_warning = res < WARNING_RES ? 1 : temp_warning;
  temp_error = res < ERROR_RES ? 1 : temp_error;
  #endif  // LED_WARNING
  
  return res;
}


#ifdef CONVERT_TO_DEG_C
/**
 * Converte the voltage to resistance and then to
 * tempurature. Write to serial with leading space.
 * y = -135.0324 + (53198.11 - -135.0324)/(1 + (x/1.027147e-12)^0.1577176)
 * Simplified:
 * y = -135 + 53333 / (1 + (x / 1.027E-12)^0.1577)
 */
#define A 72789.6
#define B 0.1656
#define C (1.634E-13)
#define D (-129.9)
 
void print_temp(int v) {
  float res = raw_to_resistance(v);
  float temp = D + A / (1 + pow((res / C), B));

  #ifdef LCD_SCREEN
  sprintf(lcd_str, "T: %s", String(temp).c_str());
  lcd.printstr(lcd_str);
  #endif  // LCD_SCREEN
  
  Serial.print(',');
  Serial.print(' ');
  Serial.print(temp);
}


#else

/**
 * Converte the voltage to resistance and print
 * to serial with leading space.
 */
void print_res(int v) {
  float res = raw_to_resistance(v);

  
  #ifdef LCD_SCREEN
  sprintf(lcd_str, "T: %s", String(res).c_str());
  lcd.printstr(lcd_str);
  #endif  // LCD_SCREEN
  
  Serial.print(',');
  Serial.print(' ');
  Serial.print(res);
}


#endif  // CONVERT_TO_DEG_C

///////////////////////////// Init //////////////////////////////
/**
 * Setup the Serial port for outputing information.
 * Dynamically setup pins depending on the number enabled.
 * Number of avalable pins are 1, 2, and 4. Will not
 * include code that's not needed.
 */
void setup() {
  Serial.begin(BAUD_RATE);

  // Set up pins for voltage input
  pinMode(INPUT_PIN_1, INPUT);
  pins[0] = INPUT_PIN_1;
  
  #if (NUM_THERMISTORS > 1)
  pinMode(INPUT_PIN_2, INPUT);
  pins[1] = INPUT_PIN_2;
  
  #if (NUM_THERMISTORS > 2)
  pinMode(INPUT_PIN_3, INPUT);
  pinMode(INPUT_PIN_4, INPUT);
  pins[2] = INPUT_PIN_3;
  pins[3] = INPUT_PIN_4;
  #endif
  #endif

  // Set up LEDs for tempurature warning (if included)
  #ifdef LED_WARNING
  pinMode(OK_PIN_POS, OUTPUT);
  pinMode(OK_PIN_NEG, OUTPUT);
  pinMode(WARNING_PIN_POS, OUTPUT);
  pinMode(WARNING_PIN_NEG, OUTPUT);
  pinMode(ERROR_PIN_POS, OUTPUT);
  pinMode(ERROR_PIN_NEG, OUTPUT);

  digitalWrite(OK_PIN_POS, HIGH);
  digitalWrite(OK_PIN_NEG, LOW);
  digitalWrite(WARNING_PIN_POS, HIGH);
  digitalWrite(WARNING_PIN_NEG, HIGH);
  digitalWrite(ERROR_PIN_POS, HIGH);
  digitalWrite(ERROR_PIN_NEG, HIGH);
  #endif  // LED_WARNING

  // Set up LCD screen
  #ifdef LCD_SCREEN
  lcd.init();
  lcd.backlight();
  lcd_ticks = 0;
  lcd_print = true;

  lcd.setCursor(1,0);
  lcd.printstr("Tempurature");
  lcd.setCursor(1,1);
  lcd.printstr("Sensor Init");
  lcd.setCursor(0,0);
  delay(1000);
  #endif // LCE_SCREEN
}


/////////////////////////////// Loop //////////////////////////
/**
 * Record a before and after timestamp for averaging time 
 * difference. Read the voltages and then convert to 
 * resistance or tempurature (depending on setup). Wait
 * DELAY_MS for the next measurement. Allows for data rate
 * setup.
 */
void loop() {
  // Prints start and end time for averaging timing
  Serial.print(millis());
  
  // Measure raw data first to get measurements as close as possible.
  // No loops or jumps for faster code.
  voltage_raw[0] = analogRead(pins[0]);
  #if (NUM_THERMISTORS > 1)
  voltage_raw[1] = analogRead(pins[1]);
  #if (NUM_THERMISTORS > 2)
  voltage_raw[2] = analogRead(pins[2]);
  voltage_raw[3] = analogRead(pins[3]);
  #endif
  #endif
  
  Serial.print(',');
  Serial.print(' ');
  Serial.print(millis());
  
  // Initialize warnings to check if tempurature is getting to high
  #ifdef LED_WARNING
  temp_ok = 1;
  temp_warning = 0;
  temp_error = 0;
  #endif  // LED_WARNING

  // Convert raw data into desired information, print to serial for future analysis.
  for (int i = 0; i < NUM_THERMISTORS; i++) {
    // Convert voltage to correct value to print
    #ifdef CONVERT_TO_DEG_C
    print_temp(voltage_raw[i]);
    #else
    print_res(voltage_raw[i]);
    #endif

    #ifdef LCD_SCREEN
    if (i + 1 == NUM_THERMISTORS / 2)
      lcd.setCursor(0,1);
    #endif  // LCD_SCREEN
  }

  // Turn on LEDs if there are warnings/errors
  #ifdef LED_WARNING
  digitalWrite(OK_PIN_NEG, temp_ok ? LOW : HIGH);
  digitalWrite(WARNING_PIN_NEG, temp_warning ? LOW : HIGH);
  digitalWrite(ERROR_PIN_NEG, temp_error ? LOW : HIGH);
  #endif  // LED_WARNING

  Serial.print("\n");

  #ifdef LCD_SCREEN
  lcd_ticks = (lcd_ticks + 1) % LCD_REFRESH_TICKS;
  lcd_print = !lcd_ticks;
  lcd.setCursor(0,0);
  #endif  // LCD_SCREEN

  // Time delay between measurements.
  delay(DELAY_MS);
}
