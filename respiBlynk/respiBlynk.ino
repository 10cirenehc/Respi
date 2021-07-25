
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

//define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address

CCS811 mySensor(CCS811_ADDR);

// you should get Auth Token in the Blynk App.
// go to the Project Settings (nut icon).
char auth[] = "qoMQKz1WeZXd0LzxQsM54Xgv1UkAmq87";

// your WiFi credentials.
// set password to "" for open networks.
char ssid[] = "ORBI19";
char pass[] = "MAji0220";

// create variable of type BlynkTimer, see more details below
BlynkTimer timer;

unsigned long previousMillis = 0;    // will store last time DHT was updated
const long interval = 9000;  

// these correspond to the pins on your NodeMCU

const int dZero = 16; //D0
const int dOne = 5; //D1
const int dTwo = 4;   //D2
const int dThree = 0;  //D3
const int dFour = 2; //D4
const int dFive = 14;    //D5
const int dSix = 12;    //D6

DHT dht(dFour, DHTTYPE);

void setup() {
  // declare pins as inputs and outputs
  pinMode(dZero, INPUT);
  pinMode(dOne, INPUT);
  pinMode(dTwo, INPUT);
  //pinMode(dThree, INPUT);
  pinMode(dFour, INPUT);
  pinMode(dFive, INPUT);
  pinMode(dSix, INPUT);
  
  // start the serial connection
  Serial.begin(115200);
  mySensor.begin();
  Wire.begin(); //Inialize I2C Hardware

  if (mySensor.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing..."); //If you are getting this error, check that the connections with the sparkfun are tight. You will need to adjust a little
    while (1)
      ;
  }

    // starts the connection with Blynk using the data provided at the top (Wi-Fi connection name, password, and auth token)
  Blynk.begin(auth, ssid, pass);

  // a timer function which is called every 1000 millisecond. Note that it calls the function myTimerEvent, which in turn send the currentDistance to the Blynk server
  timer.setInterval(10000L, myTimerEvent); // setup a function to be called every second

//  // wait for serial monitor to open
//  while(! Serial);
//
//  Serial.print("Connecting to Adafruit IO");
//
//  // connect to io.adafruit.com
//  io.connect();
//
//  // wait for a connection
//  while(io.status() < AIO_CONNECTED) {
//    Serial.print(".");
//    delay(500);
//  }
//
//  // we are connected
//  Serial.println();
//  Serial.println(io.statusText());

}

int button;
float sdsLow, sdsHigh;

float t; //float variables allow for decimals
float h;

float tempCO2;
float tempVOC;

void loop() {

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
//  io.run();

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
  
  sdsLow = pulseIn(dFive, LOW);
  sdsHigh = pulseIn(dSix, LOW);
  Serial.print("PM 2.5: ");
  Serial.println(sdsLow);
  Serial.print("PM 10: ");
  Serial.println(sdsHigh);

//  buttonf->save(button);
//  tempf->save(t);
//  humidf->save(h);
//  spkfunTVOCf->save(tempVOC);
//  spkfunCO2f->save(tempCO2);
//  sdsLowf->save(sdsLow);
//  sdsHighf->save(sdsHigh);

  
  delay(10000);

}

void myTimerEvent(){
  Blynk.virtualWrite(V1, t); // send data to app
  Blynk.virtualWrite(V2, h); // send data to app
  Blynk.virtualWrite(V3, tempVOC); // send data to app
  Blynk.virtualWrite(V4, tempCO2); // send data to app
  Blynk.virtualWrite(V5, button); // send data to app
  
}

//// these correspond to the pins on your NodeMCU
//#define trigPin 14    // D5 in Node MCU
//#define echoPin 12    // D6 in Node MCU
//const int greenPin = 5; // D1 in Node MCU
//const int redPin = 4;   // D2 in Node MCU
//
//void setLED(int distance) {
//  if (distance >= 0 && distance < 12) {
//    digitalWrite(redPin, HIGH);
//    digitalWrite(greenPin, LOW);
//  } else if (distance >= 12 && distance < 40) {
//    digitalWrite(redPin, LOW);
//    digitalWrite(greenPin, HIGH);
//  } else if (distance>=40) {
//    digitalWrite(redPin, LOW);
//    digitalWrite(greenPin, LOW);
//  }
//}
//
//long getDistance()
//{
//  long duration, distance;
//  
//  // clears the trigPin
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  
//  // sets the trigPin on HIGH state for 10 micro seconds
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  
//  // reads the echoPin, returns the sound wave travel time in microseconds
//  duration = pulseIn(echoPin, HIGH);
//  
//  // calculating the distance
//  distance = (duration / 2) / 29.1;
//
//  // changes the LED color based on the distance calculated previously
//  setLED(distance);
//  
//  // the following code can be useful in order to debug any problems with your ping sensor
//  // in order to use it, uncomment the code below
//  //  Serial.print("Duration: ");
//  //  Serial.print(duration);
//  //  Serial.print(" Distance: ");
//  //  Serial.println(distance);
//  
//  return distance;
//}
//
//void myTimerEvent()
//{
//  float currentDistance = getDistance(); // try not to send >10 values/secondz
//  Blynk.virtualWrite(V5, currentDistance); // send data to app
//}
//
//void setup()
//{
//  // sets up pins
//  pinMode (trigPin , OUTPUT );
//  pinMode (echoPin , INPUT );
//  pinMode (redPin, OUTPUT);
//  pinMode (greenPin, OUTPUT);
//  
//  // opens serial monitor at 9600 baud
//  Serial.begin(9600);
//
//  // starts the connection with Blynk using the data provided at the top (Wi-Fi connection name, password, and auth token)
//  Blynk.begin(auth, ssid, pass);
//
//  // a timer function which is called every 1000 millisecond. Note that it calls the function myTimerEvent, which in turn send the currentDistance to the Blynk server
//  timer.setInterval(1000L, myTimerEvent); // setup a function to be called every second
//}
//
//void loop()
//{
//  // runs the code
//  Blynk.run();
//  timer.run();
//}
