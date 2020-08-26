//#define SERIAL_DEBUG
# define DEV_ID "T01"
//# define DEV_ID "T02"

#include <ESP8266WiFi.h>

#include <DHT.h>
#include <DHT_U.h>
#define DHT_PIN 13
DHT dht(DHT_PIN, DHT11);

float temp = 25.0;
float humidity = 10.0;

#include <WiFiUdp.h>
#include "BetterNTP.h"
#include <time.h>
#include "wifi_creds.h"
const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASS;

const char *ntpServer = "192.168.86.9";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer);

IPAddress serverIP = IPAddress(192, 168, 86, 9);
#define UDP_PORT 19086
WiFiUDP serverUDP;

#define BATTERY_PIN A0
uint16_t battery = 0;

void setup() {
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay(1);
    
  Serial.begin(115200);

#if defined(SERIAL_DEBUG)
  while (!Serial);
#endif  

  Serial.println("Initializing temp sensor");
  dht.begin();
  readDHT();

  readBattery();

  // Now we wake up the Wifi and send our message
  WiFi.forceSleepWake();
  delay(1);
  WiFi.persistent( false );

  
  WiFi.mode(WIFI_STA);
  Serial.print("Initializing WIFI to talk to SSID ");
  Serial.print(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
    //WiFi.begin(ssid, pass);
  }
  Serial.println("!");

  Serial.print("Received IP: ");
  Serial.println(WiFi.localIP());

  // Start NTP client
  timeClient.begin();
  timeClient.forceUpdate();
 
  serverUDP.begin(UDP_PORT);
  refreshNTP();
  registerServer();
  delay( 1 );

  WiFi.disconnect( true );
  delay( 1 );

  ESP.deepSleep(50e6, WAKE_RF_DISABLED);
}

void loop() {
}

void readDHT() {
  temp = dht.readTemperature();
  humidity = dht.readHumidity();
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("C Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
}

void readBattery() {
  battery = analogRead(BATTERY_PIN);
  Serial.print("Battery: ");
  Serial.print(battery*10*1.0/1024);
  Serial.println("V");
}

size_t writeTimeInfo(WiFiUDP &udp, uint32_t epoch, uint32_t epoch_micros) {
  size_t bytes_written = 0;
  bytes_written += udp.write((const uint8_t *) &epoch, sizeof(epoch));
  bytes_written += udp.write((const uint8_t *) &epoch_micros, sizeof(epoch_micros));
  return bytes_written;
}


void registerServer() {
  int rc;
  rc = serverUDP.beginPacket(serverIP, UDP_PORT);
  if (rc == 0) {
    Serial.println("UNABLE TO ALLOCATE PACKET for sendBuffer()");
    return;
  }
  if (serverUDP.write('T') != 1) Serial.println("Unable to write cmd");
  if (serverUDP.write((byte *) DEV_ID, 3) != 3) Serial.println("Unable to write id");
  if (writeTimeInfo(serverUDP, timeClient.getEpochTime(), timeClient.getEpochMicros()) != 8) Serial.println("Unable to write time");
  if (serverUDP.write((const uint8_t *) &battery, sizeof(battery)) != sizeof(battery)) Serial.println("Unable to write battery status");
  uint16_t temp_int = (uint16_t) ((temp + 273.15)*10.0);
  if (serverUDP.write((const uint8_t *) &temp_int, sizeof(temp_int)) != sizeof(temp_int)) Serial.println("Unable to write temperature");
  uint16_t humid_int = (uint16_t) (humidity*10.0);
  if (serverUDP.write((const uint8_t *) &humid_int, sizeof(humid_int)) != sizeof(humid_int)) Serial.println("Unable to write humidity");
  if (serverUDP.endPacket() == 0) Serial.println("Unable to send packet");
  yield(); // give the chip time to send the packet

  Serial.println("Reinit finished");
}

void refreshNTP() {
  uint32_t beforeEpoch = timeClient.getEpochTime();
  uint32_t beforeMicros = timeClient.getEpochMicros();
  uint32_t sysBeforeMicros = micros();

  if ( timeClient.forceUpdate() ) {
    uint32_t afterEpoch = timeClient.getEpochTime();
    uint32_t afterMicros = timeClient.getEpochMicros();
    uint32_t sysAfterMicros = micros();

    int32_t skewMicros = 1000000 * (afterEpoch - beforeEpoch) + afterMicros - beforeMicros - (sysAfterMicros - sysBeforeMicros);

    String msg = "Time skew ";
    msg += skewMicros;
    msg += " microseconds";
    Serial.println(msg.c_str());

    char microStr[8];
    msg = "Last offset: ";
    if (timeClient.offsetNegative()) {
      msg += "-";
    }
    msg += timeClient.getOffset();
    snprintf(microStr, 8, ".%06d", timeClient.getOffsetMicros());
    microStr[7] = '\0';
    msg += microStr;
    msg += " seconds";
    Serial.println(msg.c_str());

    msg = "Last delay: ";
    msg += timeClient.getDelay();
    snprintf(microStr, 8, ".%06d", timeClient.getDelayMicros());
    microStr[7] = '\0';
    msg += microStr;
    msg += " seconds";
    Serial.println(msg.c_str());

  } else {
    Serial.println("Time update failed");
  }
}
