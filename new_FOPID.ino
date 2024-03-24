#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TRSensors.h"
#include <Wire.h>
#include <math.h>
#include "Complex.h"

#define PWMA 6
#define AIN2 A0
#define AIN1 A1
#define PWMB 5
#define BIN1 A2
#define BIN2 A3
#define PIN 7
#define NUM_SENSORS 5
#define OLED_RESET 9
#define OLED_SA0 8
#define Addr 0x20
#define ECHO 2
#define TRIG 3

#define beep_on PCF8574Write(0xDF & PCF8574Read())
#define beep_off PCF8574Write(0xDF & PCF8574Read())

Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

TRSensors trs = TRSensors();

unsigned int sensorValues[NUM_SENSORS];
unsigned int last_proportional = 0;
unsigned int position;

long lambda = 0.45;
long mu = 0.5;

// int h = 1;
// int L = 10;
// int k = 3;

uint16_t i, j;
byte value;
unsigned long lasttime = 0;
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

void PCF8574Write(byte data);
byte PCF8574Read();
uint32_t Wheel(byte WheelPos);

long proportional;
long integral;
long derivative;
long err;

//int u;
//int fopid(long err, int proportional, int integral, int derivative, long lambda, long mu, int h, int L, int k);
//int cal(int a, int err, int k, int m);

int Distance = 0;
int Speed = 20;
int Distance_test();


void forward();
void right();
void left();
void stop();
void evade();

void setup() 
{
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,0);
  display.println("WaveShare");
  display.setCursor(10,25);
  display.println("AlphaBot2");
  display.setTextSize(1);
  display.setCursor(10,55);
  display.println("Press to calibrate");
  display.display();

  while(value != 0xEF)
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
  }

  pinMode(PWMA,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN2,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  RGB.begin();
  RGB.setPixelColor(0,0x00FF00);
  RGB.setPixelColor(1,0x00FF00);
  RGB.setPixelColor(2,0x00FF00);
  RGB.setPixelColor(3,0x00FF00);
  //RGB.show();
  delay(500);
  analogWrite(PWMA,50);
  analogWrite(PWMB,50);
  for (int i = 0; i < 100; i++)
  {
    if(i < 25 || i >= 75)
    {
      digitalWrite(AIN1,HIGH);
      digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,LOW);
      digitalWrite(BIN2,HIGH);
    }
    else
    {
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);
      digitalWrite(BIN2,LOW);
    }
    trs.calibrate();
  }
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN2,LOW);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,LOW);
  RGB.setPixelColor(0,0x0000FF);
  RGB.setPixelColor(1,0x0000FF);
  RGB.setPixelColor(2,0x0000FF);
  RGB.setPixelColor(3,0x0000FF);
  //RGB.show();

  value = 0;
  while(value != 0xEF)
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
    position = trs.readLine(sensorValues)/200;
    display.clearDisplay();
    display.setCursor(0,25);
    display.println("Calibration Done !!!");
    display.setCursor(0,55);
    for (int i = 0; i < 21; i++)
    {
      display.print('_');
    }
    display.setCursor(position*6,55);
    display.print("**");
    display.display();
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,0);
  display.println("ALPHABOT");
  display.setTextSize(3);
  display.setCursor(40,30);
  display.println("GO!");
  display.display();

  delay(500);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);

  Wire.begin();
  pinMode(ECHO, INPUT);
  pinMode(TRIG,OUTPUT);

}

void loop(){

  position = trs.readLine(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println(position);

  int proportional = (int)position - 2000;
  int derivative = pow((proportional - last_proportional),mu);
  integral = pow((integral+proportional),-lambda);
  last_proportional = proportional;

  int power_difference = proportional/20 + integral/10000 + derivative*10; 
  //fopid(position, proportional, integral, derivative, lambda, mu, h, L, k); //h=step size, L=memory length, k=current discrete time value
  const int maximum = 40;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < - maximum)
    power_difference = - maximum;
    Serial.println(power_difference);

  if (power_difference < 0)
  {
    analogWrite(PWMA,maximum + power_difference);
    analogWrite(PWMB,maximum);
  }
  else
  {
    analogWrite(PWMA,maximum);
    analogWrite(PWMB,maximum - power_difference);
  }

  if (sensorValues[1] > 900 && sensorValues[2] > 900 && sensorValues[3] > 900)
  {
    analogWrite(PWMA,maximum);
    analogWrite(PWMB,maximum);
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,HIGH); 
    digitalWrite(BIN2,LOW);  
  }
  /*
  if(millis() - lasttime > 200)
  {
    lasttime = millis();
    for(i=0;i<RGB.numPixels();i++)
    {
      RGB.setPixelColor(i, Wheel(((i * 256 / RGB.numPixels()) + j) & 255));
    }
    //RGB.show();
    if(j++ > 256*4) j = 0;
  }*/
}

uint32_t Wheel(byte WheelPos){
  if(WheelPos < 85){
    return RGB.Color(WheelPos * 50, 255 - WheelPos * 50 , 0);
  }else if (WheelPos < 170){
    WheelPos -= 85;
    return RGB.Color(255 - WheelPos * 50, 0, WheelPos * 50);
  }else{
    WheelPos -= 170;
    return RGB.Color(0,WheelPos * 50 , 255 - WheelPos * 50);
  }
}

void PCF8574Write(byte data)
{
  Wire.beginTransmission(Addr);
  Wire.write(data);
  Wire.endTransmission();
}

byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(Addr,1);
  if(Wire.available()){
    data = Wire.read();
  }
  return data;
}

int cal(int a, int err, int k, int m){
    int out = 1*(k-0);
    int q_re = 1;
    for (int count = 1;  count <= m-1; count++){
        int q_j = (1 - (1+a)/j)*q_re;
        out = out + q_j*(k-count);
        q_re = q_j;
}
}    

// int fopid(long err, int proportional, int integral, int derivative, long lambda, long mu, int h, int L, int k)
// {
//   int m = min(k,L);
//   err = proportional;
//   u = (proportional/20)*k + (integral/10000)*h*lambda*cal(-lambda,err,k,m) + (derivative*10)*pow(h,-mu)*cal(mu,err,k,m);
// }

//obstacle avoidance

void forward()
{
  analogWrite(PWMA,Speed);
  analogWrite(PWMB,Speed);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW);  
  digitalWrite(BIN2,HIGH); 
}

void backward()
{
  analogWrite(PWMA,Speed);
  analogWrite(PWMB,Speed);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH); 
  digitalWrite(BIN2,LOW);    
}

void right()
{
  analogWrite(PWMA,50);
  analogWrite(PWMB,50);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH); 
  digitalWrite(BIN2,LOW);    
}

void left()
{
  analogWrite(PWMA,50);
  analogWrite(PWMB,50);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,HIGH);    
}

void stop()
{
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,LOW);    
}

// int Distance_test()
// {
//   digitalWrite(TRIG,LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIG,HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG,LOW);
//   float Fdistance = pulseIn(ECHO,HIGH);
//   Fdistance = Fdistance/58;
//   return(int)Fdistance;
// }
