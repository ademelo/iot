/*

  HelloWorld.ino + Wifi

*/

#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <MyScreen.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <TimeLib.h>
#include <WiFiUdp.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// NTP Servers:
static const char ntpServerName[] = "fr.pool.ntp.org";//"us.pool.ntp.org";//"0.fr.pool.ntp.org";
const int timeZone = 2;
WiFiUDP Udp;
unsigned int localPort = 8888; // local port to listen for UDP packets

time_t getNtpTime();
void sendNTPpacket(IPAddress &address);
//time_t prevDisplay = 0; // when the digital clock was displayed

/*==========================================================
                    Début MQTT
  ==========================================================*/

#define XIVELY_BROKER             "broker.xively.com"
#define XIVELY_BROKER_PORT        443
// Enter your Xively credentials and topic
#define ACCOUNT_ID                "5b1607f6-bd17-4043-a079-42b03a7c9358"
#define XIVELY_DEVICE_ID          "ecdd9340-67bf-4b30-af7e-7181839fae6e"
#define XIVELY_DEVICE_CREDENTIALS "fermdBnUlkvfSp0UpKOIq+FLhiqC245eC2sekfIUZoY="

#define ROOT_TOPIC                "xi/blue/v1/"
#define ROOT_ACCOUNT_TOPIC        ROOT_TOPIC ACCOUNT_ID
#define LED_TOPIC                 ROOT_ACCOUNT_TOPIC "/d/" XIVELY_DEVICE_ID "/led"
#define TEMPERATURE_TOPIC         ROOT_ACCOUNT_TOPIC "/d/" XIVELY_DEVICE_ID "/temperature"
#define EVENT_TOPIC               ROOT_ACCOUNT_TOPIC "/d/" XIVELY_DEVICE_ID "/event"
#define LED_RBG_TOPIC             ROOT_ACCOUNT_TOPIC "/d/" XIVELY_DEVICE_ID "/ledRGB"

// Only change this if you have a newer certificate fingerprint
const char* fingerprint = "8A 39 7B 12 C6 53 8F AA 3E 6E 73 70 34 26 41";
// Setup the MQTT secure client class by passing in the WiFi client and MQTT server and login details.
WiFiClientSecure client;
static Adafruit_MQTT_Client mqtt(&client, XIVELY_BROKER, XIVELY_BROKER_PORT, XIVELY_DEVICE_ID, XIVELY_DEVICE_CREDENTIALS);

// Setup Channel
Adafruit_MQTT_Subscribe led = Adafruit_MQTT_Subscribe(&mqtt, LED_TOPIC, 1);   // qos=1
Adafruit_MQTT_Subscribe ledRGB = Adafruit_MQTT_Subscribe(&mqtt, LED_RBG_TOPIC, 1);   // qos=1

// Function declarations
void MQTT_connect();
void verifySecure();
float getTemperature();
void getBlinkCountFromServer(const char* stringFromServer);
void getRGBColorFromServer(char* stringFromServer);
void publishTemperature(float temperature);
void publishEventPushButton();
void pushButtonEventRaised();
void blink(int times);

volatile boolean handRaised = false;
// last color of the LED
volatile int ledRed = 0;

unsigned long lastMillisLoop = 0;
const unsigned long timeFrameForTemperature = 5000; // 5 secondes

/*==========================================================
                    Fin MQTT
  ==========================================================*/

MyScreen myScreen("MyScreen message: it works!!! :)");


#define ONE_WIRE_BUS D1
OneWire oneWire(ONE_WIRE_BUS); //Bus One Wire sur la pin 2 de l'arduino
DallasTemperature sensors(&oneWire); //Utilistion du bus Onewire pour les capteurs
DeviceAddress sensorDeviceAddress; //Vérifie la compatibilité des capteurs avec la librairie

const char* ssid = "NETGEAR-IOT";//"G5_5710";//"Bbox-45A7D94D";
const char* password = "PocIot2017!";//"spleenlex0879";//"341E1A2D234272C6527426A441D4CF";

int ledPin = 13; // GPIO13
WiFiServer server(80);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


void setup(void) {
  u8g2.begin();

  Serial.begin(115200);

  sensors.begin(); //Activation des capteurs
  sensors.getAddress(sensorDeviceAddress, 0); //Demande l'adresse du capteur à l'index 0 du bus
  sensors.setResolution(sensorDeviceAddress, 12); //Résolutions possibles: 9,10,11,12

  /*pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);*/

  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  //TODO: DateTime Init
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(1);

  // Finds the onboard LED and sets it to off
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  /*==========================================================
                      Début MQTT
    ==========================================================*/
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  // verify SSL certificate origin
  verifySecure();
  blink(1);
  mqtt.subscribe(&led);
  mqtt.subscribe(&ledRGB);
  /*==========================================================
                      Fin MQTT
    ==========================================================*/
}


/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() {
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void printMessage(const char *message){
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_profont22_tf);//u8g2_font_ncenB14_tf); // choose a suitable font
  u8g2.drawStr(20,30,message); // write something to the internal memory
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(5, 20, 0x2600);	/* dec 9731/hex 2603 Snowman */
  u8g2.drawHLine(0,35,128);
  //u8g2.drawStr(10,62,"08/02/1979");
  u8g2.setFont(u8g2_font_amstrad_cpc_extended_8f);
  u8g2.setCursor(0,48);
  u8g2.print(hour(), DEC); u8g2.print('h');
  u8g2.print(minute(), DEC); u8g2.print(':');
  u8g2.print(second(), DEC);
  u8g2.setCursor(0,60);
  u8g2.print(day(), DEC); u8g2.print('/');
  u8g2.print(month(), DEC); u8g2.print('/');
  u8g2.print(year(), DEC);
  u8g2.sendBuffer();          // transfer internal memory to the display
  //u8g2.drawHLine(0,50,90);
  //delay(1000);
}


void loop(void) {

  sensors.requestTemperatures(); //Demande la température aux capteurs
  float temperatureDegC = sensors.getTempCByIndex(0);
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(temperatureDegC, 6, 2, result); // Leave room for too large numbers!
  //strcat(result, 'C°');
  char unity[4] = "C";
  char concatstr[12];
  sprintf(concatstr, "%s%s", result, unity);
  printMessage(concatstr);

  delay(5000);

  /*==========================================================
                      Début MQTT
    ==========================================================*/
  unsigned long currentMillis;

  // Ensure the connection to the MQTT server is alive
  MQTT_connect();

  // Get temperature every x seconds
  currentMillis = millis();
  if (currentMillis > lastMillisLoop + timeFrameForTemperature) {
    lastMillisLoop =   currentMillis;
    publishTemperature(temperatureDegC);
  }
  // Subscribe to channel and apply changes.
  // Messages in the channel should be 0 or 1
  /*Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(0))) {
    if (subscription == &led)
      getBlinkCountFromServer((const char*) led.lastread);
    if (subscription == &ledRGB)
      getRGBColorFromServer((char *)ledRGB.lastread);
  }*/
  /*==========================================================
                      Fin MQTT
    ==========================================================*/
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }

  Serial.println("MQTT Connected!");
}

void verifySecure() {

  const char* host = XIVELY_BROKER;

  Serial.print("Connecting to ");
  Serial.println(host);

  if (! client.connect(host, XIVELY_BROKER_PORT)) {
    Serial.println("Connection failed. Halting execution.");
    while(1);
  }

  // Shut down connection if host identity cannot be trusted.
  if (client.verify(fingerprint, host)) {
    Serial.println("Connection secure.");
  } else {
    Serial.println("Connection insecure! Halting execution.");
    //while(1);
  }
}

void blink(int times) {
  int count = 0;
  while (count < times) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    count++;
  }
}

void publishTemperature(float temperature) {

  char temperatureString[6];
  char temperatureTimeSerieString[255] = "";

  // convert temperature to a string with two digits before the comma and 2 digits for precision
  dtostrf(temperature, 2, 2, temperatureString);

  // Debug : send temperature to the serial console
  Serial.print("Sending temperature to Cloud: ");
  Serial.println(temperatureString);

  // Input format 1: CSV
  // CSV structure
  // Datapoints can be stored so long as they are published in the following format:
  //  Timestamp,Category,Value,String
  //  ,Temperature, value, esp8266-5ième-sud
  //strcpy(temperatureTimeSerieString, ",");
  strcat(temperatureTimeSerieString,"temperature,");
  strcat(temperatureTimeSerieString, temperatureString);
  strcat(temperatureTimeSerieString,",");
  Serial.print(temperatureTimeSerieString);
  //strcat(temperatureTimeSerieString, MAC_address);
    // TODO : build the JSON data for poc DN (see picture)


    // send temperature to the MQTT topic
  mqtt.publish(TEMPERATURE_TOPIC, temperatureTimeSerieString);

}
