#include <Adafruit_NAU7802.h>
#include <LiquidCrystal.h>
#include <Wire.h>


//LEDS
#define BLUE 13
#define GREEN 12
#define RED 11


Adafruit_NAU7802 nau;


LiquidCrystal lcd(9, 8, 5, 4, 3, 2);


const double r1 = -0.1; const double r2 = 1000;
const double g1 = 3200; const double g2 = 5000.1;
const double b1 = 500; const double b2 = 4300;

int theoretical = 1; //what its supposed to be
int measured = 1; //what were measuring

double SUM = 0;

//variables for moving average
volatile int INDEX = 0;
volatile int32_t VALUE = 0;
const int WINDOW_SIZE = 100;
volatile double READINGS[WINDOW_SIZE];
volatile double AVERAGED = 0;

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
  Wire.beginTransmission(0x2A);
  Wire.write(0b0001110);
  Wire.write(0b10000000);
  Wire.endTransmission();


  lcd.clear();


  lcd.print("Voltage:");
  lcd.setCursor(0, 1);
  lcd.print("ppm: ");
  
  cli(); //stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 33332;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set bits for 8 prescaler
  TCCR1B |= (1 << CS11);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //enable interrupts
  Serial.println("timers set up");
}

//timer interrupt when timer1 reaches interval
ISR(TIMER1_COMPA_vect){
  //VALUE = nau.read();
  SUM = SUM - READINGS[INDEX];
  READINGS[INDEX] = VALUE;
  SUM = SUM + VALUE;
  INDEX = (INDEX + 1) % WINDOW_SIZE;
}

void loop() {
  // put your main code here, to run repeatedly:
  AVERAGED = SUM / WINDOW_SIZE;

  double averageV = AVERAGED * 4.49 / pow(2, 24);

  double ppm = (exp((averageV-3.29)/-0.357)/2)*theoretical/measured;

  if (ppm > 5000) ppm = 5000;
  if (ppm < 0) ppm = 0;

  if (r1 < ppm && ppm < r2) digitalWrite(RED, 1);
  if (g1 < ppm && ppm < g2) digitalWrite(GREEN, 1);
  if (b1 < ppm && ppm < b2) digitalWrite(BLUE, 1);

  Serial.print("ADC reading:"); Serial.print(AVERAGED); Serial.print("   "); Serial.print("Voltage:");  Serial.print(averageV, 3); Serial.print("V   "); Serial.print("ppm:"); Serial.print(ppm, 0); Serial.print("ppm   ");  Serial.print("top:"); Serial.print(10000);
    Serial.print("   ");
    Serial.print("bottom:"); Serial.println(-10000);

  lcd.print("Voltage:"); lcd.print(averageV, 4); lcd.print("V"); lcd.print("          ");
  lcd.setCursor(0, 1);
  lcd.print("ppm:"); lcd.print(ppm, 0); lcd.print("ppm"); lcd.print("          ");

  digitalWrite(RED, 0); digitalWrite(GREEN, 0); digitalWrite(BLUE, 0);

}
