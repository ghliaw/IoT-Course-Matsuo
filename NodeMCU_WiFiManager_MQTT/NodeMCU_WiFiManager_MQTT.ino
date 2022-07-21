/*********
 * Test for WiFiManager:
 * If not connected, RGB LED is red. 
 * Use laptop or phone connected to the AP of this NodeMCU and start a brower to configure WiFi's parameters.
 * After being connected, RGB LED is 
*********/

#include <string.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
#include <DHT.h>
#include <PubSubClient.h>

// WiFiManager AP name
#define WM_AP_NAME  "NodeMCUWiFiConfig-ghliaw"

// constants for RGB LED
#define RGBLED_ON  false // for common anode LED
#define RGBLED_OFF true  // for common anode LED
#define PIN_RED     D5
#define PIN_BLUE    D6
#define PIN_GREEN   D7

// LED Control info
typedef enum {
  LED_STATUS_OFF = 0,
  LED_STATUS_ON,
  LED_STATUS_FLASH
} LEDStatus;
LEDStatus ledStatus;
#define LED_COLOR_WIFI_NO "red"
#define LED_COLOR_WIFI_OK "blue"
#define LED_COLOR_TEST  "green"

// contants for DHT sensor
#define DHTPIN D2
#define DHTTYPE DHT11   // DHT 11 
#define PERIOD_DHT  15000 // period for sensing temperature and humidity

// Global variables for DHT Sensor
DHT dht(DHTPIN, DHTTYPE);
unsigned long dht_last_time;

// MQTT Broker info
//IPAddress server(140, 127, 196, 119);
char server[] = "broker.hivemq.com";
int port = 1883;

// MQTT Client info
// Client ID.
// Note that a broker allows an individual client to create only one session.
// If a session is created by another client with same cliend ID, the former one will be disconnected.
// Thus, each sensor node's client must be different from each other.
// Leave client_id as an empty string here to let the program randomly decides it.   
char client_id[] = ""; 

// MQTT topics
#define TOPIC_INFO  "test220722/info"
#define TOPIC_TEMP  "test220722/sensor/temp"
#define TOPIC_HUM   "test220722/sensor/hum"
#define TOPIC_LED   "test220722/sensor/led"
#define TOPIC_LED_CONTROL "test220722/control/led"

// Clients for MQTT
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Contants and global variables for processing button press 
#define PIN_BUTTON  D1
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

/*
 * MQTT callback function
 */
void callback(char* topic, byte* payload, unsigned int length) {
  // Output incoming message to serial terminal
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // If LED Control command is incoming, change LED status
  if(!strcmp(topic, TOPIC_LED_CONTROL)) {
    switch (payload[0]) {
      case '0':
        ledStatus = LED_STATUS_OFF;
        break;
      case '1':
        ledStatus = LED_STATUS_ON;
        break;
      case '2':
        ledStatus = LED_STATUS_FLASH;
        break;
      default: {}
    }
  }
}

/*
 * MQTT conenct/reconnect function
 */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    char *p_cid;
    char tempstr[10];
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    // (if client_id is an empty string, generate a random string for it.)
    if(strlen(client_id)==0) {
      long r;
      randomSeed(millis());
      r = random(1, 10000);
      sprintf(tempstr,"liaw%04d", r);
      p_cid = tempstr;
    }
    else p_cid = client_id;
    Serial.print("Connecting to MQTT broker, client id = ");
    Serial.println(p_cid);
    if (client.connect(p_cid)) {
      Serial.println("MQTT connected");
      // Once connected, publish an announcement...
      client.publish(TOPIC_INFO,"sensor node ready ...");
      sprintf(tempstr,"%s", String((int)ledStatus).c_str());
      client.publish(TOPIC_LED, tempstr);
      // ... and resubscribe
      client.subscribe(TOPIC_LED_CONTROL);
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*
 *  Set color of RGB LED
 */
void rgbColor(char *color)
{
  if(strcmp(color, "red")==0) {
    digitalWrite(PIN_RED, RGBLED_ON);
    digitalWrite(PIN_GREEN, RGBLED_OFF);
    digitalWrite(PIN_BLUE, RGBLED_OFF);
  }
  else if(strcmp(color, "green")==0) {
    digitalWrite(PIN_RED, RGBLED_OFF);
    digitalWrite(PIN_GREEN, RGBLED_ON);
    digitalWrite(PIN_BLUE, RGBLED_OFF);
  }
  else if(strcmp(color, "blue")==0) {
    digitalWrite(PIN_RED, RGBLED_OFF);
    digitalWrite(PIN_GREEN, RGBLED_OFF);
    digitalWrite(PIN_BLUE, RGBLED_ON);
  }
  else if(strcmp(color, "off")==0) {
    digitalWrite(PIN_RED, RGBLED_OFF);
    digitalWrite(PIN_GREEN, RGBLED_OFF);
    digitalWrite(PIN_BLUE, RGBLED_OFF);
  }
}

void setup() {
  // Initial hardware setting
  Serial.begin(115200);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  // Initially, set LED color for no wifi
  rgbColor(LED_COLOR_WIFI_NO);
    
  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  
  // Uncomment and run it once, if you want to erase all the stored information
  //wifiManager.resetSettings();
  
  // set custom ip for portal
  //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("NodeMCUWiFiConfig");
  // or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected.");

  // After being connected, set LED color for wifi ok
  rgbColor(LED_COLOR_WIFI_OK);
  ledStatus = LED_STATUS_ON;

  // Initialize DHT & LCD
  dht.begin();

  // record the current time for DHT timer
  dht_last_time = 0;

  // Set MQTT broker
  client.setServer(server, port);
  client.setCallback(callback);
}

void loop(){
  static char buf[20];
  int incoming_byte; 
  float h, t;
  unsigned long current_time;

  // Check MQTT broker connection status
  // If it is disconnected, reconnect to the broker
  if (!client.connected()) {
    reconnect();
  }
  
  current_time = millis();
    
  /*
   * If PERIOD_DHT timeout is reached, read temperature and humidity once and output
   */
  if(current_time - dht_last_time >= PERIOD_DHT) {
    h = dht.readHumidity();
    t = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (!isnan(t) && !isnan(h)) 
    {
        // Print Temperature & Humidity data on Serial port
        Serial.print(t,1);
        Serial.print(",");
        Serial.println(h,1); 

        // Print Temperature & Humidity data on Serial port
        Serial.print("Temp: ");
        Serial.print(t, 1);
        Serial.print(" C, ");
        Serial.print("Hum : ");
        Serial.print(h, 1);
        Serial.println(" \%");
    }
    // pubilsh to MQTT broker
    sprintf(buf,"%s", String((float)t, 2).c_str());
    client.publish(TOPIC_TEMP, buf);
    sprintf(buf,"%s", String((float)h, 2).c_str());
    client.publish(TOPIC_HUM, buf);
    // update last time value
    dht_last_time = current_time;
  }

  /* Processing button press (including debounce): 
   * Once the button is pressed and then released, LED is toggled and its status is published to MQTT broker
  */
  // read the state of the switch into a local variable:
  int reading = digitalRead(PIN_BUTTON);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      Serial.print("Button State = ");
      Serial.println(buttonState);

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        if (ledStatus==LED_STATUS_OFF) ledStatus = LED_STATUS_ON;
        else if (ledStatus==LED_STATUS_ON) ledStatus = LED_STATUS_OFF;

        // pubilsh to MQTT broker
        sprintf(buf,"%s", String((int)ledStatus).c_str());
        client.publish(TOPIC_LED, buf);
      }
    }
  }

  // Control LED according to ledStatus
  switch (ledStatus) {
    case LED_STATUS_OFF:
      rgbColor("off");
      break;
    case LED_STATUS_ON:
      rgbColor(LED_COLOR_TEST);
      break;
#if 0
    case LED_STATUS_FLASH:
      current_time = millis();
      if (LED_FLASH_PERIOD < (current_time - led_last_time)) {
        led_toggle();
        led_last_time = current_time;
      }
      break;
#endif
    default: {}
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

  // Keep MQTT process on going
  client.loop();
}
