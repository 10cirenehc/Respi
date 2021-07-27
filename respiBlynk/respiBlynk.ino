
//BLYNK CONFIGURATION

#define APP_DEBUG
// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

#define BLYNK_PRINT Serial

// include libraries required for Blynk and NodeMCU communication
#include <ESP8266WiFi.h>
#include <BlynkSimpleStream.h>


#define BLYNK_TEMPLATE_ID "TMPLC03EIzmT"
#define BLYNK_DEVICE_NAME "RohanRespi"
char auth[] = "WAivy6qAUKwk7n5UY1s8E0zJO7OvWR5a";

// your WiFi credentials.
// set password to "" for open networks.
char ssid[] = "Rohan's 11 Pro";
char pass[] = "hello123";

// "Pliploo"
// "12345678"
WiFiClient wifiClient;

//#define BLYNK_DEBUG



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


DHT dht(dFour, DHTTYPE);

#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
 
int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;         //define PM10 value of the air detector module
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

void setup() {
  // declare pins as inputs and outputs
  pinMode(dZero, INPUT);
  pinMode(dOne, INPUT);
  pinMode(dTwo, INPUT);
  pinMode(dFour, INPUT);
  pinMode(dFive, INPUT);
  pinMode(dSix, INPUT);
  pinMode(dSeven, OUTPUT);
  
  // start the serial connection
  Serial.begin(9600);
  delay(10);
  mySensor.begin();
  Wire.begin(); //Inialize I2C Hardware

  if (mySensor.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing..."); //If you are getting this error, check that the connections with the sparkfun are tight. You will need to adjust a little
    while (1)
      ;
  }

    // starts the connection with Blynk using the data provided at the top (Wi-Fi connection name, password, and auth token
  connectWiFi();
  connectBlynk();
  Blynk.begin(wifiClient, auth);
  
  // a timer function which is called every 1000 millisecond. Note that it calls the function myTimerEvent, which in turn send the currentDistance to the Blynk server
  timer.setInterval(30000L, myTimerEvent); // setup a function to be called every 30 seconds

}

int button;
//float sdsLow, sdsHigh;

float t; //float variables allow for decimals
float h;

float tempCO2;
float tempVOC;

void loop() {

  if(Serial.find(0x42)){    //start to read when detect 0x42
    Serial.readBytes(buf,LENG);
 
    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        PM10Value=transmitPM10(buf); //count PM10 value of the air detector module 
      }           
    } 
  }
 
  static unsigned long OledTimer=millis();  
    if (millis() - OledTimer >=1000) 
    {
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
      Serial.println();
    }
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    return;
  }

  // Reconnect to Blynk Cloud
  if (!wifiClient.connected()) {
    connectBlynk();
    return;
  }

  Blynk.run();
  timer.run();

  
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
    
  }
  
  
  //Measure temperature and humidity.  If the functions returns
  //true, then a measurement is available. 
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;
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
  }
  
  //digitalWrite(dFour, LOW);
  if(digitalRead(button)==HIGH){
    button=1;
  }

  if(t<minTemp || t>maxTemp || h<minHum || h>maxHum || tempVOC>minTVOC || PM01Value>minPM1 || PM2_5Value>minPM2_5 || PM10Value>minPM10){

    for(int i = 0; i<5; i++){
      tone(dSeven, 1000, 150);
      delay(3000);
    }
    
  }

   tList[counter%3] = t;
   hList[counter%3] = h;

   tempCO2List[counter%3] = tempCO2;
   tempVOCList[counter%3] = tempVOC;

   PM01List[counter%3] = PM01Value;
   PM2_5List[counter%3] = PM2_5Value;
   PM10List[counter%3] = PM10Value;

  delay(30000);

}

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
float average(float a[3]){
  float sum = a[0] + a[1] + a[2];
  return sum/3;
}

bool connectBlynk()
{
  wifiClient.stop();
  return wifiClient.connect(BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT);
}

// This function tries to connect to your WiFi network
void connectWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);

  if (pass && strlen(pass)) {
    WiFi.begin((char*)ssid, (char*)pass);
  } else {
    WiFi.begin((char*)ssid);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}
