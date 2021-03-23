/*************************************************************************
  Company: ITG TECHNOLOGY JSC.
  Title:    Program for ATmega328 (UNO, Mini, Nano). Varroc project.
  Author:   Diepdv <dvdiep@3serp.com.vn>
  File:     arduino_iic.c
  Software: ArduinoIDE
**************************************************************************/
/////////// PIN MAPPING ////////////
/*
  GY-33 ----- Pro mini 5v 16m
  VCC   ----- VCC
  CT    ----- A5
  DR    ----- A4
  GND   ----- GND
  S0    ----- GND
  /////////////////////
  Button ----- Pro mini
  Select ----- 2
  Signal ----- 3
*/

//////////// COMMAND FUNCTION SEND TO COM PORT /////////////
/*
  uart
  send 'b' ----- Calibration GY-33
  send 'a' ----- GY33 send data
  send 'Y' ----- ACK
*/

//////////// ADDRESS SAVE DATA IN EEPROM /////////////
/*
  EEPROM Address
  0 - Red value array
  1 - Green value array
  2 - Blue value array
*/

#include "i2cmaster.h"
#include <EEPROM.h>
#include <TimerOne.h>

///////////// VARIABLE /////////////////////
#define uint16_t unsigned int
#define INTERVAL 3000
#define TIMER_US 100000     // 100ms set timer duration in microseconds 
#define TICK_COUNTS 30      // Coount 3s

typedef struct {
  uint16_t Red;
  uint16_t Green;
  uint16_t Blue;
  uint16_t Clear;
} RGB;
unsigned char Re_buf;
volatile unsigned char sign = 0;
volatile unsigned char check_ack = 0;
volatile int countMode = 0;                       // 0(Run) -- 1(Get Red value sample) -- 2(Get Green value sample) -- 3(Get Blue value sample)
volatile int enableGetSample = 0;                 // 0(enable) -- 1(disable)
// unsigned char dataSend = 0;
volatile long tick_count = TICK_COUNTS;           // Counter for 2 seconds
volatile bool in_long_isr = false;                // True if in long interrupt
unsigned char lastColor = 0, currentColor = 0;    // 1(Red) -- 2(Green) -- 0(Other)
unsigned char redVal = 0, greenVal = 0, blueVal = 0;
RGB rgb;
uint16_t CT = 0, Lux = 0;
byte color = 0;
unsigned char r_rgb_data[3] = {0};
unsigned char g_rgb_data[3] = {0};
unsigned char b_rgb_data[3] = {0};
unsigned char rgb_data[3] = {0};
volatile unsigned long time_now = 0;

void setup() {
  Serial.begin(9600);
  // Init led indicator
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  // iic
  i2c_init();
  // Init timer 1
  Timer1.initialize(TIMER_US);                    // Initialise timer 1
  Timer1.attachInterrupt(timerIsr);               // attach the ISR routine here
  // Init external interrupt for 2 button
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), selectMode, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), selectGetSample, FALLING);
  // getSampleSignal();
  // Read data R, G, B from EEPROM
  readData(r_rgb_data, g_rgb_data, b_rgb_data);
  // Serial.print("RR_EEP:");
  // Serial.print(r_rgb_data[0]);
  // Serial.print(" RG_EEP:");
  // Serial.print(r_rgb_data[1]);
  // Serial.print(" RB_EEP:");
  // Serial.println(r_rgb_data[2]);
  // Serial.print("GR_EEP:");
  // Serial.print(g_rgb_data[0]);
  // Serial.print(" GG_EEP:");
  // Serial.print(g_rgb_data[1]);
  // Serial.print(" GB_EEP:");
  // Serial.println(g_rgb_data[2]);
  // Serial.print("BR_EEP:");
  // Serial.print(b_rgb_data[0]);
  // Serial.print(" BG_EEP:");
  // Serial.print(b_rgb_data[1]);
  // Serial.print(" BB_EEP:");
  // Serial.println(b_rgb_data[2]);
  // Init system
  ledIndicator(countMode);
  delay(10);
}

// --------------------------
// timerIsr() 100ms interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr() {
  if (!(--tick_count)) {                           // Count to 2S
    tick_count = TICK_COUNTS;                      // Reload
    if (check_ack == 0) {
      if (currentColor != lastColor) {
        // // Serial.print(check_ack);
        if (currentColor == 1) Serial.print("F");
        else if (currentColor == 2) Serial.print("T");
      }
    }
    else if (check_ack == 1) {
      lastColor = currentColor;
      check_ack = 0;
    }
  }
}

// Function clear value to EEPROM
void clearData(int address) {
  //  for (int i = 0 ; i < 3 ; i++) {
  //    if(EEPROM.read(i) != 0)                     // Skip already "empty" addresses
  //    {
  //      EEPROM.write(i, 0);                       // Write 0 to address i
  //    }
  //  }
  EEPROM.write(address, 0);                       // Write 0 to address i
}

// Function write R, G, B data to EEPROM
void writeData(int address, byte value) {
  EEPROM.write(address, value);
}

// Function read R, G, B data to EEPROM
void readData(byte *rs, byte *gs, byte *bs) {
  *rs = EEPROM.read(0);
  *(rs + 1) = EEPROM.read(1);
  *(rs + 2) = EEPROM.read(2);
  *gs = EEPROM.read(3);
  *(gs + 1) = EEPROM.read(4);
  *(gs + 2) = EEPROM.read(5);
  *bs = EEPROM.read(6);
  *(bs + 1) = EEPROM.read(7);
  *(bs + 2) = EEPROM.read(8);
}

// Delay function
void delay_ms(int ms) {
  // delay 200ms
  time_now = millis();
  while (millis() < time_now + (unsigned long)ms) {
    // Delay [period] ms
  }
}

// Led status for each mode
void ledIndicator(int numberMode) {
  switch (numberMode) {
    case 0: {     // Running
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        digitalWrite(7, LOW);
      }; break;
    case 1: {     // Retrieve red signal sampling
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
        digitalWrite(6, LOW);
        digitalWrite(7, LOW);
      }; break;
    case 2: {     // Retrieve green signal sampling
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, HIGH);
        digitalWrite(7, LOW);
      }; break;
    case 3: {     // Retrieve blue signal sampling
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        digitalWrite(7, HIGH);
      }; break;
    default: break;
  }
}

// Led status
void ledStatus(int numberMode) {
  switch (numberMode) {
    case 0: {     // Running
        for (int i = 0; i < 5; i++) {
          // // Serial.print("1...");
          digitalWrite(4, LOW);
          delay_ms(200);
          digitalWrite(4, HIGH);
          delay_ms(200);
        }
      }; break;
    case 1: {     // Retrieve red signal sampling
        for (int i = 0; i < 5; i++) {
          // // Serial.print("2...");
          digitalWrite(5, LOW);
          delay_ms(200);
          digitalWrite(5, HIGH);
          delay_ms(200);
        }
      }; break;
    case 2: {     // Retrieve green signal sampling
        for (int i = 0; i < 5; i++) {
          digitalWrite(6, LOW);
          delay_ms(200);
          digitalWrite(6, HIGH);
          delay_ms(200);
        }
      }; break;
    case 3: {     // Retrieve blue signal sampling
        for (int i = 0; i < 5; i++) {
          digitalWrite(7, LOW);
          delay_ms(200);
          digitalWrite(7, HIGH);
          delay_ms(200);
        }
      }; break;
    default: break;
  }
}

// Function excute select mode
void selectMode() {
  if(enableGetSample == 0) {
    countMode++;
    if (countMode > 3) {
      countMode = 0;
    }
    if (countMode == 0) {
      enableGetSample = 0;
    }
    ledIndicator(countMode);
    // enableGetSample = 1;
  }
}

// Function get sample
void selectGetSample() {
  enableGetSample = 1;
}

// ---------------------------
// This function gets a signal sampling and save to flash
// Call it when want to adjust
// ---------------------------
void getSampleSignal() {
  if (enableGetSample != 0) {
    // Led status
    ledStatus(countMode);
    // Get sample excute
    if (countMode == 0) {
      readData(r_rgb_data, g_rgb_data, b_rgb_data);
      // Serial.print("RR_EEP:");
      // Serial.print(r_rgb_data[0]);
      // Serial.print(" RG_EEP:");
      // Serial.print(r_rgb_data[1]);
      // Serial.print(" RB_EEP:");
      // Serial.println(r_rgb_data[2]);
      // Serial.print("GR_EEP:");
      // Serial.print(g_rgb_data[0]);
      // Serial.print(" GG_EEP:");
      // Serial.print(g_rgb_data[1]);
      // Serial.print(" GB_EEP:");
      // Serial.println(g_rgb_data[2]);
      // Serial.print("BR_EEP:");
      // Serial.print(b_rgb_data[0]);
      // Serial.print(" BG_EEP:");
      // Serial.print(b_rgb_data[1]);
      // Serial.print(" BB_EEP:");
      // Serial.println(b_rgb_data[2]);
      enableGetSample = 0;
    }
    else {
      unsigned char dataSample[4] = {0};
      int _redVal = 0, _greenVal = 0, _blueVal = 0;
      // Read R, G, B data
      for (int i = 0; i < 20; i++) {
        iic_read(0x0c, dataSample, 3);
        if (i >= 5 && i < 15) {
          _redVal = _redVal + dataSample[0];
          _greenVal = _greenVal + dataSample[1];
          _blueVal = _blueVal + dataSample[2];
        }
        delay_ms(10);
      }
      // Get sample value for each value of the color
      _redVal = _redVal / 10;
      _greenVal = _greenVal / 10;
      _blueVal = _blueVal / 10;
      // Reset enable get sample
      enableGetSample = 0;
      // Write data to EEPROM
      switch (countMode) {
        case 1: {     // red
            // EEPROM.write(0, 0);
            EEPROM.write(0, _redVal);
            EEPROM.write(1, _greenVal);
            EEPROM.write(2, _blueVal);
            // Serial.print("R_SAMPLE:");
            // Serial.print(_redVal);
            // Led status
            ledStatus(countMode);
          }; break;
        case 2: {     // green
            // EEPROM.write(1, 0);
            // EEPROM.write(1, _greenVal);
            EEPROM.write(3, _redVal);
            EEPROM.write(4, _greenVal);
            EEPROM.write(5, _blueVal);
            // Serial.print(" G_SAMPLE:");
            // Serial.print(_greenVal);
            // Led status
            ledStatus(countMode);
          }; break;
        case 3: {     // blue
            // EEPROM.write(0, 0);
            // EEPROM.write(2, _blueVal);
            EEPROM.write(6, _redVal);
            EEPROM.write(7, _greenVal);
            EEPROM.write(8, _blueVal);
            // Serial.print(" B_SAMPLE:");
            // Serial.println(_blueVal);
            // Led status
            ledStatus(countMode);
          }; break;
        default: break;
      }
    }
  }
}

void loop() {
  // unsigned char data[9] = {0};
  unsigned char data[4] = {0};  // size = 4 for r,g,b color data
  if (!sign) {
    /* Read raw data
      iic_read(0x00, data, 8);
      rgb.Red = (data[0] << 8) | data[1];
      rgb.Green = (data[2] << 8) | data[3];
      rgb.Blue = (data[4] << 8) | data[5];
      rgb.Clear = (data[6] << 8) | data[7];
      // Serial.print("Red: ");
      // Serial.print(rgb.Red);
      // Serial.print(",Green: ");
      // Serial.print(rgb.Green);
      // Serial.print(",Blue");
      // Serial.print(rgb.Blue);
      // Serial.print(",Clear");
      // Serial.println(rgb.Clear);
      iic_read(0x08, data, 4);
      Lux = (data[0] << 8) | data[1];
      CT = (data[2] << 8) | data[3];
      // Serial.print("CT:");
      // Serial.print(CT);
      // Serial.print(",Lux:");
      // Serial.println( Lux);
    */
    /* Read R,G,B data */
    iic_read(0x0c, data, 3);
    rgb_data[0] = data[0];  // Red
    rgb_data[1] = data[1];  // Green
    rgb_data[2] = data[2];  // Blue
    // Detect Color
    if (rgb_data[0] > (r_rgb_data[0] - 5) && rgb_data[0] < (r_rgb_data[0] + 5)  \
        && rgb_data[1] > (r_rgb_data[1] - 5) && rgb_data[1] < (r_rgb_data[1] + 5) \
        && rgb_data[2] > (r_rgb_data[2] - 5) && rgb_data[2] < (r_rgb_data[2] + 5) ) {              // Red
      // Send to RS232 with result check is fail
      currentColor = 1;
      // dataSend = 'F';
    }
    if (rgb_data[0] > (g_rgb_data[0] - 5) && rgb_data[0] < (g_rgb_data[0] + 5)  \
        && rgb_data[1] > (g_rgb_data[1] - 5) && rgb_data[1] < (g_rgb_data[1] + 5) \
        && rgb_data[2] > (g_rgb_data[2] - 5) && rgb_data[2] < (g_rgb_data[2] + 5) ) {              // Green
      // Send to RS232 with result check is pass
      currentColor = 2;
      // dataSend = 'T';
    }
    // Serial.print("R:");
    // Serial.print(rgb_data[0]);
    // Serial.print(" G:");
    // Serial.print(rgb_data[1]);
    // Serial.print(" B:");
    // Serial.println(rgb_data[2]);
    /* Read color data
      iic_read(0x0f, data, 1);
      color = data[0];
      // Serial.print(",color:");
      // Serial.println( color, HEX);
    */
  }
  if (sign == 1) {
    iic_read(0x10, &data[8], 1);
    i2c_start_wait(0xb4);
    i2c_write(0x10);
    i2c_write(0x31);
    // i2c_write((data[8]|0x01));
    i2c_stop();
    sign = 3;
  }

  // Get sample signal
  getSampleSignal();
  enableGetSample = 0;
  // delay(300);
  // delay 200ms
  time_now = millis();
  while (millis() < time_now + 300) {
    // Delay [period] ms
  }
}

void iic_read(unsigned char add, unsigned char *data, unsigned char len) {
  i2c_start_wait(0xb4);
  i2c_write(add);
  i2c_start_wait(0xb5);
  while (len - 1) {
    *data++ = i2c_readAck();
    len--;
  }
  *data = i2c_readNak();
  i2c_stop();
}

void serialEvent() {
  while (Serial.available()) {
    Re_buf = (unsigned char)Serial.read();
    if (Re_buf == 'a') sign = 0;
    if (Re_buf == 'b') sign = 1;
    if (Re_buf == 'Y') check_ack = 1;
    Re_buf = 0;
  }
}
