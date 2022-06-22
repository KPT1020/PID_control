#include <Wire.h> // specify use of Wire.h library
#define ASD1115 0x48
//#define ASD1115_1 0x49

unsigned int val = 0;
unsigned int val_1 = 0;
byte writeBuf[3];
byte buffer[3];
byte setting[3];
byte channel [4];
unsigned long previoustime;
int period = 100; // 1/Frequency *1000ms

const int gain = 4.096; //gain one=4.096;gain two= 2.048;

const int ledPin = 26;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
const int zone = 5000;
const int dutyCycle = 128;

int16_t error, lastE, output;
long currentT, previousT;
float differE, integrateE, elapsedT;

float calculateHeating(int input);
int16_t PIDsystem(int input);

void setup()   {
  //SETTING ADS INTERNAL PROPERTY//
  setting[0] = 1;  setting[1] = 0; setting[2] = 0b11100101;
  //setting[0] = 1;  setting[1] = 0b11110010; setting[2] = 0b11100101; // use it with setting array to control two ads with different input channel
  channel[0] = 0b11000010 ; channel[1] = 0b11010010; channel[2] = 0b11100010; channel[3] = 0b11110010 ;

  // config register is 1
  // bit 15 flag bit for single shot
  // Bits 14-12 input selection:
  // 100 >> ANC0    101 >> ANC1
  // 110 >> ANC2    111 >> ANC3
  // Bits 11-9 Amp gain: (Default to 010 here 001 P19)
  // 000 >> FS = ±6.144V    100 >> FS = ±0.512V
  // 001 >> FS = ±4.096V    101 >> FS = ±0.256V
  // 010 >> FS = ±2.048V    110 >> FS = ±0.256V
  // 011 >> FS = ±1.024V    111 >> FS = ±0.256V
  // Bit 8 Operational mode of the ADS1115:
  // 0 : Continuous conversion mode
  // 1 : Power-down single-shot mode (default)
  // Bits 7-5 data rate default to 100 for 128SPS(Sample per second):
  // 000 >> 8SPS   100 >> 128SPS (default)
  // 001 >> 16SPS    101 >> 250SPS
  // 010 >> 32SPS    110 >> 475SPS
  // 011 >> 64SPS    111 >> 860SPS
  // Bits 4-0  comparator functions see spec sheet.
  //DONES SETTING//

    Serial.begin(115200);
  Wire.begin(); // begin I2C
  pinMode(ledPin, OUTPUT);


  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, dutyCycle);
  Serial.print("Hello:");

}

void loop() {
  int rewrite;
  float HResist;
  if (millis() > previoustime + period) {
    previoustime = millis();
    Wire.beginTransmission(ASD1115);  // ADC
    setting[1] = channel[1];
    Wire.write(setting[0]); Wire.write(setting[1]); Wire.write(setting[2]);
    Wire.endTransmission();
    delay(5);
    readADS(ASD1115);
  }


  //delay(10);

  //  Serial.print("Raw is: "); Serial.println(adc1);
  //  HResist = calculateHeating(adc1);
  //  Serial.print("Resistance:  "); Serial.println(HResist);
  //  rewrite = PIDsystem(adc1);
  //  Serial.print("Rewrite: "); Serial.println(rewrite);
  //  calculateHeating(adc1);
  //  ledcWrite(ledChannel, rewrite);
  //  delay(10);
} // end loop
//
//int16_t PIDsystem(int input) {
//  float Kp = 1;
//  float Ki = 3;
//  float Kd = 0.001;
//  int16_t setPoint = 20;
//
//  currentT = millis();
//  elapsedT = currentT - previousT;
//  elapsedT = (elapsedT) / 1000;
//  //  Serial.print("Correct T: "); Serial.println(elapsedT, 4);
//  //  Serial.print("input value: "); Serial.println(input);
//  error = setPoint - input;
//  //  Serial.print("error: "); Serial.println(error); /*Serial.print(" ")*/;
//  integrateE += error * elapsedT;
//  //    Serial.print("integrate error: "); Serial.print(integrateE, 4); Serial.print(" ");
//  differE = error - lastE;
//  differE = differE / elapsedT;
//  //  Serial.print("differential error: "); Serial.println(differE, 4);
//  output =  (int)Kp * error + Ki * integrateE + Kd * differE;
//   Serial.print("output value:"); Serial.println(output);
//  if (output < 0) {
//    output = 0;
//  }
//  else if (output > 255) {
//    output = 255;
//  }
//  previousT = currentT;
//  lastE = error;
//  return output;
//}
//
//float calculateHeating(int input) {
//  const int constant = 8000;
//  const float resistRef = 0.05;
//  float power = 1.2;
//  float resistance;
//
//  resistance = ((constant * power * resistRef) / input) - resistRef;
//  Serial.print("Resistance: ");Serial.println(resistance,4);
//}

int readADS(byte asd) {
  buffer[0] = 0; // pointer
  Wire.beginTransmission(asd);
  Wire.write(buffer[0]);  // pointer
  Wire.endTransmission();

  Wire.requestFrom(asd, 2);
  buffer[1] = Wire.read(); buffer[2] = Wire.read();
  Wire.endTransmission();

  val = buffer[1] << 8 | buffer[2]; // convert display results
  switch (setting[1]) {
    case 0b11000010:
      Serial.print("ADS 1_0: "); Serial.print(val); Serial.println("\t\t");
      break;
    case 0b11010010:
      Serial.print("ADS 1_1: "); Serial.print(val); Serial.println("\t\t");
      break;
    case 0b11100010:
      Serial.print("ADS 1_2: "); Serial.print(val); Serial.println("\t\t");
      break;
    case 0b11110010:
      Serial.print("ADS 1_3: "); Serial.print(val); Serial.println("\t\t");
      break;
  }
}
