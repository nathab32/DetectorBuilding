  int orpPin = 1;
  int redPin = 2;
  int greenPin = 3;
  int bluePin = 4;
  
  float tempC = 25;
  float tempK = tempC + 273.15;
  int refPpm = 6000;
  int limitOne = 100;
  int limitTwo = 400;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:


  float orpVoltage = analogRead(orpPin) * 0.0049;
  float orpReading = refPpm * pow(10, (0-(orpVoltage*96485.3)/(8.315*tempK*2.303)));
  
  Serial.println("Voltage: ");
  Serial.print(orpVoltage);

  Serial.println("Salt concentration: ");
  Serial.print(orpReading);
  Serial.print("ppm");

  if (orpReading < limitOne){
      analogWrite(redPin, 100);
  }
  else if (orpReading < limitTwo){
      analogWrite(greenPin, 100);
  }
  else{
      analogWrite(bluePin, 100);
  }
}
