/************************************
    scl-----A5   Sda-----A4
*************************************/
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
#include<LiquidCrystal.h> 
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
LiquidCrystal lcd(13,12,11,10,9,8); //RS,EN,D4,D5,D6,D7    // initialize the library with the numbers of the interface pins
#define buzzer 7
#define relay A0
#define soss 6
int sos=0;
#define trigPin1 A3       // us1
#define echoPin1 A2
float lattitude, longitude;
float a[2];
float *p;
SoftwareSerial gpsSerial(4, 5); // gps 2
SoftwareSerial gsmSerial(2, 3); // gsm 11
TinyGPSPlus gps;
 long duration, distance, sensor1;
 int counter=0;
void setup() 
{
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Vehicle safety");
  pinMode(buzzer,OUTPUT);
  pinMode(relay,OUTPUT);
  pinMode(soss,INPUT_PULLUP);
  digitalWrite(buzzer,LOW);
  digitalWrite(relay,LOW); 
  //lcd.clear();
  pinMode(buzzer,OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  Serial.begin(9600);
  delay(1000);
  gpsSerial.begin(9600);
  delay(1000);
  gsmSerial.begin(9600);
  delay(1000);
  Serial.print("—Tracking–");
  Serial.print("**Location**");
  gsmSerial.println("AT+CNMI=2,2,0,0,0");
  delay(300);
  Serial.print("Initializing……");
  delay(300);
  Serial.print("System Ready  ");
  delay(500);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
   Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
   delay(500);
  }
  lcd.clear();
  checkSettings();
}
void checkSettings()
{
 Serial.println();  
 Serial.print(" * Sleep Mode:            ");
 Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
 Serial.print(" * Clock Source:          ");
 switch(mpu.getClockSource())
 {
  case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
  case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
  case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
  case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
  case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
  case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
  case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
 }
 Serial.print(" * Accelerometer offsets: ");
 Serial.print(mpu.getAccelOffsetX());
 Serial.print(" / ");
 Serial.print(mpu.getAccelOffsetY());
 Serial.print(" / ");
 Serial.println(mpu.getAccelOffsetZ());
 Serial.println();
}
void loop()
{
 ultrasensor(trigPin1, echoPin1);
 sensor1 = distance;
 sensor1=sensor1*1.05;
 Serial.print(" sensor1 = ");
 Serial.print(sensor1);
 if(sensor1<25)
 {
  digitalWrite(buzzer,HIGH);
  delay(200);
  digitalWrite(buzzer,LOW);
 }
 Vector rawAccel = mpu.readRawAccel();
 Vector normAccel = mpu.readNormalizeAccel();
 Serial.print(" Xnorm = ");
 Serial.print(normAccel.XAxis);
 Serial.print(" Ynorm = ");
 Serial.print(normAccel.YAxis);
 Serial.print(" Znorm = ");
 Serial.println(normAccel.ZAxis);
 Serial.print("counter=");
 Serial.println(counter);
 sos=digitalRead(soss);
 if(sos==LOW)
 {
  digitalWrite(buzzer,HIGH);
  SendMessage();
  digitalWrite(buzzer,LOW);
 }
 if(normAccel.XAxis>7)
 {
  digitalWrite(buzzer,HIGH);
  SendMessage();
  digitalWrite(buzzer,LOW);
 }
 else if(normAccel.XAxis<-7)
 {
  digitalWrite(buzzer,HIGH);
  SendMessage();
  digitalWrite(buzzer,LOW);
 }
 else if(normAccel.YAxis>7)
 {
  digitalWrite(buzzer,HIGH);
  SendMessage();
  digitalWrite(buzzer,LOW);
 } 
 else if(normAccel.YAxis<-7)
 {
  digitalWrite(buzzer,HIGH);
  SendMessage();
  digitalWrite(buzzer,LOW);
 }
 else
 { 
  
 } 
 
// if(counter>20)
// {
//  counter=0;
//  digitalWrite(buzzer,HIGH);
//  delay(500);
//  digitalWrite(buzzer,LOW);
//  SendMessage1(); 
// }
  delay(100);
// counter=counter+1;
} 
float *get_gps()
{
  gpsSerial.listen();
  Serial.println("INSIDE get_gps");
  while (1)
  {
    while (gpsSerial.available() > 0)
    {
     gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated())
    {
     Serial.print("LAT=");
     Serial.println(gps.location.lat(), 6);
     Serial.print("LONG=");
     Serial.println(gps.location.lng(), 6);
     lattitude = gps.location.lat();
     longitude = gps.location.lng();
     break;
    }
  }
  a[0] = lattitude;
  a[1] = longitude;
  return a;
}

void SendMessage()
 {
  gsmSerial.println("AT+CMGF=1");  //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  gsmSerial.println("AT+CMGS=\"+919212602861\"\r");  // Replace x with mobile number
  delay(1000);
  gsmSerial.println("I Am Hurt");  // The SMS text you want to send
  delay(1000);
  p = get_gps();
  gsmSerial.listen();
  Serial.print("Your Position is : ");
  gsmSerial.print("position is : ");
  Serial.print("LATTITUDE=");
  Serial.print(*p, 6);
  gsmSerial.print("LATTITUDE=");
  gsmSerial.print(*p, 6);
  gsmSerial.print(",");   // The SMS text you want to send
  Serial.print("LONGITUDE=");
  Serial.print(*(p + 1), 6); 
  gsmSerial.print("LONGITUDE=");
  gsmSerial.print(*(p + 1), 6);  // The SMS text you want to send
  delay(100);
  gsmSerial.println((char)26);
  delay(3000);
}
void SendMessage1()
 {
  gsmSerial.println("AT+CMGF=1");  //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  gsmSerial.println("AT+CMGS=\"+919821605575\"\r");  // Replace x with mobile number
  delay(1000);
  gsmSerial.println("I Am safe");  // The SMS text you want to send
  delay(1000);
  p = get_gps();
  gsmSerial.listen();
  Serial.print("Your Position is : ");
  gsmSerial.print("position is : ");
  Serial.print("LATTITUDE=");
  Serial.print(*p, 6);
  gsmSerial.print("LATTITUDE=");
  gsmSerial.print(*p, 6);
  gsmSerial.print(",");   // The SMS text you want to send
  Serial.print("LONGITUDE=");
  Serial.print(*(p + 1), 6); 
  gsmSerial.print("LONGITUDE=");
  gsmSerial.print(*(p + 1), 6);  // The SMS text you want to send
  delay(100);
  gsmSerial.println((char)26);
  delay(3000);
}
void ultrasensor(int trigPin,int echoPin)
 {  
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
 }  
