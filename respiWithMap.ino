
#define BLYNK_PRINT Serial

// include libraries required for Blynk and NodeMCU communication
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <dht_nonblocking.h> //this library was written for the sensor
//you must upload the library zip file included in the Lab 2 folder
//#define DHT_SENSOR_TYPE DHT_TYPE_11 //notice our sensor number here
#define DHTTYPE    DHT11     // DHT 11
#include <DHT.h>

#include <Wire.h> //I have no idea what this for, but it makes it work

#include "SparkFunCCS811.h" //Make sure you install this library it will not work without it

#define CCS811_ADDR 0x5B //Default I2C Address
//#define CCS811_ADDR 0x5A //Alternate I2C Address

CCS811 mySensor(CCS811_ADDR);

// you should get Auth Token in the Blynk App.
// go to the Project Settings (nut icon).
char auth[] = "KhlWjGuWfQCcTJ5l4wUboEf_zT7gCmjX";

// your WiFi credentials.
// set password to "" for open networks.
char ssid[] = "Rohan's 11 Pro";
char pass[] = "hello123";

// create variable of type BlynkTimer, see more details below
BlynkTimer timer;

unsigned long previousMillis = 0;    // will store last time DHT was updated
const long interval = 9000;  

// these correspond to the pins on your NodeMCU

const int dZero = 16; //D0
const int dOne = 5; //D1
const int dTwo = 4;   //D2
const int dFour = 2; //D4
const int dFive = 14;    //D5
const int dSix = 12;    //D6
const int dSeven = 13;    //D7
const int dEight = 15;


DHT dht(dFour, DHTTYPE);

//#include <Wire.h>
//#include <ESP8266WiFi.h>
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
 
int PM01Value;          //define PM1.0 value of the air detector module
int PM2_5Value;         //define PM2.5 value of the air detector module
int PM10Value;         //define PM10 value of the air detector module
char temperatureFString[6];
char dpString[6];
char humidityString[6];
char pressureString[7];
char pressureInchString[6];

float minTemp = 10;
float maxTemp = 35;
float minHum = 20;
float maxHum = 80;
float minTVOC = 250;
float minPM1 = 12.5;
float minPM2_5 = 25;
float minPM10 = 50;

float tList[3];
float hList[3];

float tempCO2List[3];
float tempVOCList[3];

float PM01List[3];
float PM2_5List[3];
float PM10List[3];

int counter = 0;

float lat;
float lon;

WidgetMap myMap(V15);

BLYNK_WRITE(10){
    lat = param[0].asFloat();
    lon = param[1].asFloat();
}

float average(float a[]){
  float sum = a[0] + a[1] + a[2];
  return sum/3;
}

char checkValue(unsigned char *thebuf, char leng)
{  
  char receiveflag=0;
  int receiveSum=0;
 
  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;
 
  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}
int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}
//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }
//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module  
  return PM10Val;
}

int button;
//int buttonLight;
//int buttonFlag;

float t;
float h;

float tempCO2;
float tempVOC;

void myTimerEvent(){
  Blynk.virtualWrite(V0, average(tList)); // send data to app
  Blynk.virtualWrite(V1, average(hList)); // send data to app
  Blynk.virtualWrite(V2, average(tempVOCList)); // send data to app
  Blynk.virtualWrite(V3, average(tempCO2List)); // send data to app
  Blynk.virtualWrite(V4, button); // send data to app
  Blynk.virtualWrite(V5, average(PM01List)); // send data to app
  Blynk.virtualWrite(V6, average(PM2_5List)); // send data to app
  Blynk.virtualWrite(V7, average(PM10List)); // send data to app
}

void setup() {
  // declare pins as inputs and outputs
  Serial.begin(9600);
  
  pinMode(dZero, INPUT);
  pinMode(dOne, INPUT);
  pinMode(dTwo, INPUT);
  pinMode(dFour, INPUT);
  pinMode(dFive, OUTPUT);
  pinMode(dSix, OUTPUT);
  pinMode(dSeven, OUTPUT);
  pinMode(dEight, OUTPUT);
  // start the serial connection
  

  Blynk.begin(auth, ssid, pass);
//a timer function which is called every 1000 millisecond. Note that it calls the function myTimerEvent, which in turn send the currentDistance to the Blynk server
  timer.setInterval(5000L, myTimerEvent);// setup a function to be called every 10 seconds
  
  mySensor.begin();
  Wire.begin(); //Inialize I2C Hardware

  if (mySensor.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing..."); //If you are getting this error, check that the connections with the sparkfun are tight. You will need to adjust a little
    while (1)
      ;
  }

    // starts the connection with Blynk using the data provided at the top (Wi-Fi connection name, password, and auth token)
  
  delay(10);

  //Plot a bunch of points on the map
  int index = 0;
  lat = 1.3521;
  lon = 103.8198;
  String value = "PM1.0:" +String(10.0) + " PM2.5:" + String(25.0) + " PM10:" + String(30.2);
  myMap.location(index, lat, lon, value);

  index = 1;
  lat = 1.3451;
  lon = 103.8913;
  value = "PM1.0:" +String(43.2) + " PM2.5:" + String(12.2) + " PM10:" + String(35.2);
  myMap.location(index, lat, lon, value);

  index = 2;
  lat = 1.3110;
  lon = 103.9434;
  value = "PM1.0:" +String(4.2) + " PM2.5:" + String(5.3) + " PM10:" + String(5.7);
  myMap.location(index, lat, lon, value);

  index = 3;
  lat = 1.3050;
  lon = 103.8330;
  value = "PM1.0:" +String(2.3) + " PM2.5:" + String(10.2) + " PM10:" + String(5.4);
  myMap.location(index, lat, lon, value);

  index = 4;
  lat = 1.3814;
  lon = 103.8019;
  value = "PM1.0:" +String(1.2) + " PM2.5:" + String(3.0) + " PM10:" + String(1.3);
  myMap.location(index, lat, lon, value);

  index = 5;
  lat = 1.3549;
  lon = 103.6977;
  value = "PM1.0:" +String(40.2) + " PM2.5:" + String(20.3) + " PM10:" + String(14.5);
  myMap.location(index, lat, lon, value);

  index = 6;
  lat = 1.3890;
  lon = 103.7550;
  value = "PM1.0:" +String(30.2) + " PM2.5:" + String(22.3) + " PM10:" + String(15.6);
  myMap.location(index, lat, lon, value);
}

void loop() {
  button = 0;
  String value;
  int index = 7;

  Serial.println("");

  Blynk.run();
  timer.run();
  
if(Serial.find(0x42)){    //start to read when detect 0x42 //THIS IS THE IF STATEMENT THAT RUNS 1 TIME
    //Serial.println("Step 1");
    Serial.readBytes(buf,LENG);
    if(buf[0] == 0x4d){
      //Serial.println("Step 2");
      if(checkValue(buf,LENG)){
        PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
        //Serial.println(PM01Value);
        PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        //Serial.println(PM2_5Value);
        PM10Value=transmitPM10(buf); //count PM10 value of the air detector module 
        //Serial.println(PM10Value);
      }           
    } 
  }
 
  static unsigned long OledTimer=millis();  
  if (millis() - OledTimer >=1000){
     OledTimer=millis(); 
      
     Serial.print("PM1.0: ");  
     Serial.print(PM01Value);
     Serial.println("  ug/m3");            
    
     Serial.print("PM2.5: ");  
     Serial.print(PM2_5Value);
     Serial.println("  ug/m3");     
      
     Serial.print("PM10 : ");  
     Serial.print(PM10Value);
     Serial.println("  ug/m3");   
   }
  
  //Check to see if data is ready with .dataAvailable()
  if (mySensor.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    mySensor.readAlgorithmResults();

    //Returns calculated CO2 reading
    tempCO2 = mySensor.getCO2();
    Serial.print("CO2: ");
    Serial.println(tempCO2);
    tempVOC = mySensor.getTVOC();
    Serial.print("TVOC: ");
    Serial.println(tempVOC);
    
  } else {
    Serial.println("No data");
  }
  
//  //Measure temperature and humidity.  If the functions returns
//  //true, then a measurement is available. 
unsigned long currentMillis = millis();
  //if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    //previousMillis = currentMillis;
    // Read temperature as Celsius (the default)
    
    float newT = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //float newT = dht.readTemperature(true);
    // if temperature read failed, don't change t value
    if (isnan(newT)) {
      Serial.println("Failed to read from T DHT sensor!");
    }
    else {
      t = newT;
      Serial.print("Temperature: ");
      Serial.println(t);
    }
    // Read Humidity
    float newH = dht.readHumidity();
    // if humidity read failed, don't change h value 
    if (isnan(newH)) {
      Serial.println("Failed to read from H DHT sensor!");
    }
    else {
      h = newH;
      Serial.print("Humidity: ");
      Serial.println(h);
    }
  //}
  
  if(digitalRead(dZero)==HIGH){
    button=1;
    Serial.println("Button Pressed");
    index+=1;
    value = "VOC:" + String(tempVOC)+" PM2.5:" + String(PM2_5Value) + " PM10:" + String(PM10Value);
    myMap.location(index, lat, lon, value);
  }
    //digitalWrite(dEight, HIGH);

  if(PM01Value>minPM1 || PM2_5Value>minPM2_5 || PM10Value>minPM10){ //t<minTemp || t>maxTemp || h<minHum || h>maxHum || tempVOC>minTVOC || 

//    for(int i = 0; i<5; i++){
//      tone(dSeven, 1000, 150);
//      delay(3000);
//    }

    analogWrite(dSix, 255);
    analogWrite(dFive, 0);
    
  } else {

  analogWrite(dFive, 255);
  analogWrite(dSix, 0);
      
  }

   tList[counter%3] = t;
   hList[counter%3] = h;

   tempCO2List[counter%3] = tempCO2;
   tempVOCList[counter%3] = tempVOC;

   PM01List[counter%3] = PM01Value;
   PM2_5List[counter%3] = PM2_5Value;
   PM10List[counter%3] = PM10Value;

  counter++;

  //delay(5000);

}

BLYNK_WRITE(20){
  minTemp = map(param.asInt(), 0, 1023, -30, 70);
  Serial.print("Minimum temperature threshold changed to ");
  Serial.print(minTemp);
  Serial.println("");
}
BLYNK_WRITE(21){
  minHum = map(param.asInt(), 0, 1023, 0, 100);
  Serial.print("Minimum humidity threshold changed to ");
  Serial.print(minHum);
  Serial.println("");
}
BLYNK_WRITE(22){
  minTVOC = map(param.asInt(), 0, 1023, 0, 500);
  Serial.print("Minimum VOC threshold changed to ");
  Serial.print(minTVOC);
  Serial.println("");
}
BLYNK_WRITE(23){
  minPM1 = map(param.asInt(), 0, 1023, 0, 100);
  Serial.print("Minimum PM 1 threshold changed to ");
  Serial.print(minPM1);
  Serial.println("");
}
BLYNK_WRITE(24){
  minPM2_5 = map(param.asInt(), 0, 1023, 0, 100);
  Serial.print("Minimum PM 2.5 threshold changed to ");
  Serial.print(minPM2_5);
  Serial.println("");
}
BLYNK_WRITE(25){
  minPM10 = map(param.asInt(), 0, 1023, 0, 100);
  Serial.print("Minimum PM 10 threshold changed to ");
  Serial.print(minPM10);
  Serial.println("");
}
BLYNK_WRITE(26){
  minPM10 = map(param.asInt(), 0, 1023, 0, 100);
  Serial.print("Minimum PM 10 threshold changed to ");
  Serial.print(minPM10);
  Serial.println("");
}
