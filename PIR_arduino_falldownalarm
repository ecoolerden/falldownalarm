#include <SPI.h>
#include <WiFi101.h>
#include <Wire.h>
#include "DHT.h"
#include "arduino_secrets.h" 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// CLIENT ID
#define CLIENT_ID "XXXXXX"

//arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0; 
// your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Initialize the WiFi client library
WiFiClient client;

// server address:
char server[] = "XXXXX";

// DHT humidity/temperature sensors
#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
float h;
float t;

unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L * 1000L; // delay between updates, in milliseconds

// BME280 SENSOR
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
float temperature2;
float pressure2;
float altitude2;
float humidity2;

bool motion;

void setup() {
  // Diod on the board
   pinMode(6, OUTPUT);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  //}

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // DHT
  //dht.begin();

  // BME280
  bool status = bme.begin(0x76);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        //while (1);
    }
  
  // you're connected now, so print out the status:
  printWiFiStatus();

}

void loop() {
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  while (client.available()) {
    digitalWrite(6, HIGH);
    char c = client.read();
    Serial.write(c);
  }

  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    //httpRequest();
  }

  if(analogRead(A0) > 500)   { 
  //Сигнал с датчика движения
  Serial.println("Есть движение!"); 
  motion=1; 
    }  
  else   { 
  //Нет сигнала     
  //Serial.println("Всё тихо..."); 
  motion=0;
        } 

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
   /*
    h = dht.readHumidity();
    t = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(t) || isnan(h)) 
    {
        Serial.println("Failed to read from DHT");
    } 
    else 
    {
        Serial.print("Humidity: "); 
        Serial.print(h);
        Serial.print(" %\t");
        Serial.print("Temperature: "); 
        Serial.print(t);
        Serial.println(" *C");
    }
  */
  // READING DATA FROM BME280
  temperature2 = bme.readTemperature();
  pressure2 = bme.readPressure() / 100.0F;
  altitude2 = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity2 = bme.readHumidity();
    
// SEND DATA TO SERVER
httpRequest();
  delay(1000);

}

// this method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");
    // send the HTTP PUT request:
      client.print( "GET /arduino/data.php?");
      client.print("k="); // Специальный код, например 
      client.print(CLIENT_ID); // ID клиента на Платформе
      client.print("&");
      client.print("motion=");
      client.print(motion); // показания датчика
      client.print("&");
      // ---------------------------------------
      client.print("humidity1=");
      client.print(h); // показания датчика
      client.print("&");
      client.print("temperature1=");
      client.print(t); // показания датчика
      client.print("&");
      // ---------------------------------------
      client.print("temperature2=");
      client.print(temperature2); // показания датчика
      client.print("&");
      client.print("pressure2=");
      client.print(pressure2); // показания датчика
      client.print("&");
      client.print("altitude2=");
      client.print(altitude2); // показания датчика
      client.print("&");
      client.print("humidity2=");
      client.print(humidity2); // показания датчика
      client.print("&");
      client.print("ram=");
//      client.print(freeRam()); // показания датчика
      client.println(" HTTP/1.1");
    //client.println("GET /arduino/data.php HTTP/1.1");
    client.println("Host: ils.knopka24.ru");
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();

    // note the time that the connection was made:
    lastConnectionTime = millis();
  }
  else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}

void printWiFiStatus() {
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
