// #include <Filters.h>
// #include <AH/Timing/MillisMicrosTimer.hpp>
// #include <Filters/Notch.hpp>

#include <Adafruit_NAU7802.h>
#include <LiquidCrystal.h>
#include <Wire.h>


//LEDS
#define BLUE 13
#define GREEN 12
#define RED 11
#define NAUADDRESS 0x2A


Adafruit_NAU7802 nau;


LiquidCrystal lcd(9, 8, 5, 4, 3, 2);


const double r1 = -0.1; const double r2 = 150;
const double g1 = 151; const double g2 = 2000.1;
const double b1 = 1800; const double b2 = 4300;

int actual = 1;
int measured = 1;
double SUM = 0;


int INDEX = 0;
int32_t VALUE = 0;
const int WINDOW_SIZE = 100;
double READINGS[WINDOW_SIZE];
double AVERAGED = 0;



void setup() {
  // put your setup code here, to run once:
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);


  Serial.begin(115200);
  Serial.println("Serial started.");


  lcd.begin(16, 2);
  lcd.print("LCD started");


  if (nau.begin()) Serial.println("ADC found");
  else Serial.println("ADC not found");


  if (nau.reset()) Serial.println("ADC reset");
  else Serial.println("I2C comms issue");


  if (nau.enable(true)) Serial.println("ADC powered on");
  else Serial.println("I2C comms issue");


  if (nau.calibrate(NAU7802_CALMOD_INTERNAL)) Serial.println("ADC calibrated internally");
  else Serial.println("I2C comms issue");


  if (nau.setLDO(NAU7802_4V5)) Serial.println("LDO set to 4.5V");
  else Serial.println("I2C comms issue");


  if (nau.setGain(NAU7802_GAIN_1)) Serial.println("PGA set to 1");
  else Serial.println("I2C comms issue");


  if (nau.setRate(NAU7802_RATE_10SPS)) Serial.println("Sample rate set to 10");
  else Serial.println("I2C comms issue");

  Wire.begin();

  Wire.beginTransmission(NAUADDRESS); // select device with "beginTransmission()"
  Wire.write(0x1B); // select starting register with "write()"
  Wire.endTransmission(); // end write operation, as we just wanted to select the starting register
  Wire.requestFrom(NAUADDRESS, 1); // select number of bytes to get from the device (1 bytes in this case)
  byte register4= Wire.read(); // read from the starting register

  Serial.println(register4);

  Wire.beginTransmission(NAUADDRESS); //address of nau7802 7 bit
  Wire.write(0x1C); //power control register
  Wire.write(0b10000000); //turn on pga output bypass cap
  Wire.endTransmission();

  Wire.beginTransmission(NAUADDRESS);
  Wire.write(0x1B); //PGA control register
  Wire.write(0b00010000); //disable pga
  Wire.endTransmission();



  Wire.beginTransmission(NAUADDRESS); // select device with "beginTransmission()"
  Wire.write(0x1B); // select starting register with "write()"
  Wire.endTransmission(); // end write operation, as we just wanted to select the starting register
  Wire.requestFrom(NAUADDRESS, 1); // select number of bytes to get from the device (1 bytes in this case)
  register4= Wire.read(); // read from the starting register

  Serial.println(register4);


  lcd.clear();


  lcd.print("Voltage:");
  lcd.setCursor(0, 1);
  lcd.print("ppm: ");
}

// // Sampling frequency
// const double f_s = 1000;  // Hz
// // Notch frequency (-∞ dB)
// const double f_c = 60;  // Hz
// // Normalized notch frequency
// const double f_n = 2 * f_c / f_s;

// Sample timer
// Timer<micros> timer = std::round(1e6 / f_s);

// // Very simple Finite Impulse Response notch filter
// auto filter1 = simpleNotchFIR(f_n);      // fundamental

void loop() {
  // put your main code here, to run repeatedly:
    lcd.setCursor(0, 0);


    SUM = SUM - READINGS[INDEX];            // Remove the oldest entry from the sum
    VALUE = nau.read();                     // Read the next sensor value
    READINGS[INDEX] = VALUE;                // Add the newest reading to the window
    SUM = SUM + VALUE;                      // Add the newest reading to the sum
    INDEX = (INDEX + 1) % WINDOW_SIZE;      // Increment the index, and wrap to 0 if it exceeds the window size


    AVERAGED = SUM / WINDOW_SIZE;
 

    double averageV = AVERAGED * 4.49 / pow(2, 24);

    //double ppm = exp((2.05 - averageV) / 0.244);

    // double ppm = exp((3.25 - averageV) / 0.352);
    //double ppm = exp((3.176 - averageV) / 0.3533);
    //double ppm = exp((3.213 - averageV) / 0.35265);
    double ppm = (exp((averageV-3.29)/-0.357)/2)*1.457*actual/measured; //multiply by what they want, divide by what im getting. REMOVE THE /2
    //double ppm = exp((2.46 -averageV)/0.275);
    if (ppm > 5000) ppm = 5000;
    if (ppm < 0) ppm = 0;


    if (r1 < ppm && ppm < r2) digitalWrite(RED, 1);
    if (g1 < ppm && ppm < g2) digitalWrite(GREEN, 1);
    if (b1 < ppm && ppm < b2) digitalWrite(BLUE, 1);

    Serial.print("ADC reading:"); Serial.print(AVERAGED);
    Serial.print("   ");
    Serial.print("Voltage1:"); Serial.print(averageV, 3);
    Serial.print("V   ");
    Serial.print("ppm:"); Serial.print(ppm, 0);
    Serial.print("ppm   ");
    
    Serial.print("top:"); Serial.print(10000);
    Serial.print("   ");
    Serial.print("bottom:"); Serial.println(-10000);

    lcd.print("Voltage:"); lcd.print(averageV, 4); lcd.print("V"); lcd.print("          ");
    lcd.setCursor(0, 1);
    lcd.print("ppm:"); lcd.print(ppm, 0); lcd.print("ppm"); lcd.print("          ");

    digitalWrite(RED, 0); digitalWrite(GREEN, 0); digitalWrite(BLUE, 0);
  
}
