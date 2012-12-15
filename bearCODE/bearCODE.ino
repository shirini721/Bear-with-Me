/* Last updated on December 3
posts to bear
 */
#include <SPI.h> // needed in Arduino 0019 or later
#include <WiFi.h>
#include <Twitter.h>
#include <String.h>

boolean is_moving = false;
boolean is_stationary = false;
boolean has_been_hugged = false;
int toAdd = 0;
int q0 = 0;
int q1 = 0;
int q2 = 0;
int q3 = 0;
int q4 = 0;

int accelXPin = A0;
int accelYPin = A1;
int accelZPin = A2;

int accelXYZSum = 0;
int prev = 0;
int curr = 0;


int queueSum = 0;

String prevState = "";
String currState = "";
int status_light = 8;
int hugInPin = A3;
int touchInPin = 3;
int left_foot = 7;
int right_foot = 6;
int hugOut = 5;
int patPin = 2;
//The WiFi shield uses pins 10, 11, 12, and 13 for the SPI connection to the HDG104 module. Digital pin 4 is used to control the slave select pin on the SD card.


int touchValue = 0;
int hugValue = 0;
char ssid[] = "Hackerspace"; //  your network SSID (name) 
char pass[] = "MakingIsFun!";  
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS; // status of the wifi connection

// initialize the library instance:
WiFiClient client;

const unsigned long requestInterval = 10*1000;    // delay between requests; 30 seconds
const unsigned long requestInterval2 = 30*1000;    // delay between requests; 30 seconds

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(199,59,149,200);    // numeric IP for api.twitter.com
char server[] = "api.twitter.com";     // name address for twitter API
char server_time[]="www.timeapi.org";
const char lf=10;
const char hug[] = "hugged!";
const char pat[] = "recieved!";
const char moving[] = "on the go!";
const char resting[] = "resting!";
boolean requested;                     // whether you've made a request since connecting
unsigned long lastAttemptTime = 0;     // last time you connected to the server, in milliseconds
unsigned long lastAttemptTime2 = 0;     // last time you connected to the server, in milliseconds
unsigned long lastAttemptTime3 = 0;     // last time you connected to the server, in milliseconds

String currentLine = "";               // string to hold the text from server
String tweet = "";                     // string to hold the tweet
boolean readingTweet = false;          // if you're currently reading the tweet
//YOU ARE FRIENDLY BEAR 2, but you are tweeting as friendly bear
//Tweets as friendly__bear, needs to look at friendly__bear2
Twitter twitter("977498906-actSN1uKgqGN0xOQW5i0L6UbXoHvptsntbDW82Um");
//977498906-actSN1uKgqGN0xOQW5i0L6UbXoHvptsntbDW82Um <- for friendly__bear2
//977484727-7jVoVLVuX2CSS36su3m9Kcf1UY7COBXL1ZCsNbkH <-  for friendly__bear
//
//WiFiClient client_twitter;
char inString[50]; // string for incoming serial data
int stringPos = 0; // string index counter
boolean startRead = false; // is reading?

int sensorReading;
int knockValue;
unsigned long ledTime;

void setup() {
  // reserve space for the strings:
  currentLine.reserve(256);
  tweet.reserve(150);
  // start serial port:
  Serial.begin(9600);
  pinMode(hugOut, OUTPUT);
  pinMode(status_light, OUTPUT);
  pinMode(right_foot, OUTPUT);
  pinMode(left_foot, OUTPUT);
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  } 
  // you're connected now, so print out the status:
  ledTime = millis();
  
  printWifiStatus();
  connectToServer();

}
String currtime = "";
boolean test = true;

void send_status(){
 digitalWrite(status_light, HIGH);
 delay(100);
 digitalWrite(status_light, LOW);
delay(100);
 digitalWrite(status_light, HIGH);
 delay(100);
  digitalWrite(status_light, LOW);
  delay(100);
   digitalWrite(status_light, HIGH);
   delay(100);
    digitalWrite(status_light, LOW); 
  
}
void loop()
{
  
   touchValue = readCapacitivePin(touchInPin);
  hugValue = analogRead(hugInPin);
  
  if (touchValue >= 2 && has_been_hugged) {
sendTweet(pat);   
send_status();
digitalWrite(hugOut, LOW);
  }
  
  //Serial.println(hugValue);
  //For Blue bear -> 400-500? and you really have to squeeze harder
  if (hugValue > 1022  && millis() - lastAttemptTime3 > requestInterval2) {
 Serial.println(hugValue);
    
   lastAttemptTime3 = millis();

//turn off local hug light
digitalWrite(hugOut, LOW);
//turn off pat light
  digitalWrite(patPin, LOW);
 //  hug_reciprocated = true;
 sendTweet(hug);
    //turn on sending tweet light
send_status();  
}
  

  //send XYZSum to serversize
 
accelXYZSum = analogRead(accelXPin) + analogRead(accelYPin) + analogRead(accelZPin);
  prev = curr;
  curr = accelXYZSum;
  
  
  if ((curr - prev > 300) || (prev - curr > 300) ) {
    toAdd = 2;
  } else if ((curr - prev > 150)|| (prev - curr > 150)) {
    toAdd = 1;
  } else {
    toAdd = 0;
  }
  
  queueSum = queueSum + toAdd - q0;
  // shifts values down
  q0 = q1;
  q1 = q2;
  q2 = q3;
  q3 = q4;
  q4 = toAdd;
  
  prevState = currState;
//  Serial.println(queueSum); 
  
  if (queueSum > 11) {
    Serial.println(queueSum); 
    currState = "on the go"; 
 is_moving = true;
  } else {
    currState = "resting";
    is_stationary = true;
  }
  
  if (prevState != currState && millis() - lastAttemptTime2 > requestInterval2) {
    Serial.println("Bear is now " + currState);
 
  //  Serial.println("Bear is now " + currState);

 if (is_moving){
  
  sendTweet(moving); 
   send_status();
  is_moving = false;
 }else if(is_stationary){
 sendTweet(resting);
 send_status();
 is_stationary = false;
 }
 
 lastAttemptTime2 = millis();
  }
  
  

  
 
  
  
  
  if (client.connected()) {
    if (client.available()) {
      // read incoming bytes:
      char inChar = client.read();

      // add incoming byte to end of line:
      currentLine += inChar; 

      // if you get a newline, clear the line:
      if (inChar == '\n') {
        currentLine = "";
      } 
      // if the current line ends with <text>, it will
      // be followed by the tweet:
      if ( currentLine.endsWith("<text>")) {
        // tweet is beginning. Clear the tweet string:
        readingTweet = true; 
        tweet = "";
        // break out of the loop so this character isn't added to the tweet:
        return;
      }
      // if you're currently reading the bytes of a tweet,
      // add them to the tweet String:
      if (readingTweet) {
        if (inChar != '<') {
          tweet += inChar;
        } 
        else {
          // if you got a "<" character,
          // you've reached the end of the tweet:
          readingTweet = false;
          Serial.println("READING TWEETS");
          int index = tweet.indexOf(",");
          String first_part = tweet.substring(0,index);
          Serial.println(first_part);
          String timess = tweet.substring(index, tweet.length());
Serial.println(timess);
          if (currtime.equals(timess)){ //if there has been no update in the tweets
            
          }else{ //update in the tweets
            //parse tweets for updates
            if( first_part.indexOf("hugged") != -1 ){ 
              //do hug action on the bear
              Serial.println("hug");
              has_been_hugged = true;
              digitalWrite(hugOut, HIGH);
              //delay(2000);
              //digitalWrite(hugOut, LOW);
              
            }else if( first_part.indexOf("recieved") != -1 ){
            //do pat action on the bear
            Serial.println("pat");
              digitalWrite(patPin, HIGH);
              delay(2000);
              digitalWrite(patPin, LOW);
             }else if( first_part.indexOf("on the go") != -1 ){
               //do on the go notification
              
                digitalWrite(right_foot, HIGH);
  digitalWrite(left_foot, HIGH);
            }else if( first_part.indexOf("resting") != -1 ){
               //do stopped notification
                digitalWrite(right_foot, LOW);
  digitalWrite(left_foot, LOW);
                }
           currtime = timess;
        //  Serial.println(tweet);     
          }
          // close the connection to the server:
          //client.stop(); 
        }
      }
    }   
  }
  
  else if (!client.connected() && millis() - lastAttemptTime > requestInterval) {
    // if you're not connected, and two minutes have passed since
    // your last connection, then attempt to connect again:
    connectToServer();
  }
  
}

void connectToServer() {
  // attempt to connect, and wait a millisecond:
  Serial.println("connecting to server...");
  if (client.connect(server, 80)) {
    Serial.println("making HTTP request...");
    // make HTTP GET request to twitter:
    //CMSC838F - change this for each bear
    client.println("GET /1/statuses/user_timeline.xml?screen_name=friendly__bear&count=1 HTTP/1.1");
    client.println("Host:api.twitter.com");
    client.println("Connection:close");
    client.println();
  }
  // note the time of this connect attempt:
  lastAttemptTime = millis();
}   


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

String readPage(){
  
  //read the page, and capture & return everything between '<' and '>'
  boolean readNow = false;
  boolean firstLF = false;
  stringPos = 0;
  memset( &inString, 0, 50 ); //clear inString memory
  Serial.println("Entered readPage...");
  //return inString;
  while(true){
    //Serial.println("going in while loop...");
    if (client.available()) {
      char c = client.read();
      Serial.print(c);
      if(readNow){
        Serial.println("\nreading");
        if(c==lf){
          firstLF=false;
          readNow=false;
          client.stop();
          client.flush();
          Serial.println("Returning value...");
          return inString;
        }
        inString[stringPos] = c;
        stringPos ++;
        continue;
      }
      
      if(firstLF && c==lf){
        readNow = true;
      }
      firstLF=false;
      if(c==lf){
        firstLF=true;
      }
    }//end of if clinet.available
    else
     return "error";
  }//end of while
}//end of readPage

void sendTweet(String msg){
  Serial.println("going to send tweet");
  Serial.println(msg);
  //String time = getTimeFromInternet();
  unsigned long timeMS = millis();
  String time = String(timeMS);
  //String time="1";
  //Serial.println(time);
  
  msg= msg + "," + time;
  Serial.println(msg);
  int length = msg.length()+1;
  char msgArray[length];
  msg.toCharArray(msgArray,length);
  
  Serial.print("posting to twitter on ");
  Serial.print(time);
  Serial.println(" ...");
  
  Serial.println(msgArray);
  
  if (twitter.post(msgArray)) {
    // Specify &Serial to output received response to Serial.
    // If no output is required, you can just omit the argument, e.g.
    // int status = twitter.wait();
 //   int status = twitter.wait(&Serial);
    int status = twitter.wait();
    if (status == 200) {
      Serial.println("OK.");
    } else {
      Serial.print("failed : code ");
      Serial.println(status);
    }
  } else {
    Serial.println("connection failed.");
  }
  //free(msgArray);
}




uint8_t readCapacitivePin(int pinToMeasure) {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}

