#define SERIAL_DEBUG
#ifndef ESP8266
#define MIC_ID "001"
#else 
#define MIC_ID "002"
#endif

#if defined(ESP8266)
# include <ESP8266TimerInterrupt.h>
# include <ESP8266_ISR_Timer.h>
# include <ESP8266_ISR_Timer-Impl.h>


#define ERROR_LED_PIN 0
//# define USE_DHT 1
//# include <DHT.h>
//# include <DHT_U.h>

# include <ESP8266WiFi.h>
#else // assume Feather M0 WiFi
# include <SPI.h>
# include <WiFi101.h>
# include "RTClib.h"
# define USE_RTC 1
#define ERROR_LED_PIN 13
#endif

#include <WiFiUdp.h>
#include "BetterNTP.h"
#include <time.h>
#include "wifi_creds.h"
const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASS;

IPAddress ntpServer = IPAddress(192, 168, 86, 9);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer);
volatile time_t epochTime = 0;
volatile uint32_t epochMicros = 0;

#if defined(ESP8266)
// Init ESP8266 timer 0
ESP8266Timer ITimer;
# define CLOCK_RATE 100
#else
const uint8_t clockPin = 10; // the pin receiving the clockSignal.
# define CLOCK_RATE 8192
RTC_DS3231 rtc;
#endif

const float microsPerClockCycle = 1000000.0 / CLOCK_RATE;
volatile uint8_t ticker_ticked = 0;  // increases 1 per clock cycle, needs to be reset manually
volatile uint16_t ticker = 0;  // increases 1 per clock cycle, rolls over every second.
volatile bool ticker_rollover = false;  // has the second rollover happened? reset manually

#if defined(ESP8266)
# define MICPIN A0
#else
# define MICPIN A2
#endif

#define READ_BUF_SIZE 100
uint8_t missed = 0;  // count of how many readings got skipped.
uint16_t read_buf[READ_BUF_SIZE];
uint8_t read_buf_index = 0;
uint32_t read_bufStartEpoch = 0;
uint32_t read_bufStartMicros = 0;
IPAddress micServerIP = IPAddress(192, 168, 86, 9);
#define MIC_UDP_PORT 19086
WiFiUDP micServerUDP;
bool needReinit = true;

#if defined(USE_DHT)
# define DHTPIN 5
DHT dht(DHTPIN, DHT11);
#endif

#if !defined(ESP8266)
# define ICACHE_RAM_ATTR
#endif

void  ICACHE_RAM_ATTR clock_tick() {
  ticker++;
  ticker_ticked++;
  if (ticker >= CLOCK_RATE) {
    ticker = 0;
    ticker_rollover = true;
    epochTime++;
  }
}

bool errorFlag = false;

void LOG_ERROR(const char *message) {
#if defined(SERIAL_DEBUG)
  Serial.println(message);
#endif
  errorFlag = true;
}

void setup() {
  Serial.begin(115200);

#if defined(SERIAL_DEBUG)
  while (!Serial);
#endif

#if defined(USE_DHT)
  Serial.println("Initializing temp sensor");
  dht.begin();
#endif

#if !defined(ESP8266)
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8, 7, 4, 2);
#endif

  // Check for the presence of the shield.  Really only applies to
  // Feather M0 boards, but will work for the Huzzah as well.
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

#if defined(ESP8266)
  WiFi.mode(WIFI_STA);
#endif  
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
  
#if defined(ESP8266)
  if (ITimer.attachInterruptInterval((uint32_t) microsPerClockCycle, clock_tick))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or interval");
#endif

  // Start NTP client
  timeClient.begin();

#if defined(USE_RTC)
  if (! rtc.begin()) {
    Serial.println("Can't find RTC");
    Serial.flush();
    abort();
  }

  // If the RTC isn't initialized or has lost power, we need to initialize it.
  // Replace this with NTP code once we've got it working.
  //if (rtc.lostPower()) {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //initializeRtcFromNtp();
  //}
  initializeRtcFromNtp();

  pinMode(clockPin, INPUT_PULLUP);
  rtc.writeSqwPinMode(DS3231_SquareWave8kHz);
  attachInterrupt(digitalPinToInterrupt(clockPin), clock_tick, FALLING);
#endif

  analogReadResolution(12);
  
  //pinMode(MICPIN, INPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  setClockFromNTP();

  micServerUDP.begin(MIC_UDP_PORT);
  registerMicServer();
}

void loop() {
  while (!ticker_rollover) {
    while (ticker_ticked == 0) delayMicroseconds(10);
    if (ticker_ticked > 1) {
      missed += ticker_ticked - 1;
      for (uint8_t i = 1; i < ticker_ticked; i++) {
        read_buf[read_buf_index] = 0;
        read_buf_index = (read_buf_index + 1) % READ_BUF_SIZE;
        if (read_buf_index == 0) {
          sendBuffer();
        }
      }
    }
    ticker_ticked = 0; // reset ticker counter;
    // take measurement
    uint16_t reading = analogRead(MICPIN);
    read_buf[read_buf_index] = reading;
    read_buf_index = (read_buf_index + 1) % READ_BUF_SIZE;
    if (read_buf_index == 0) {
      sendBuffer();
    }
  }
  ticker_rollover = false; // reset rollover flag
  // put your main code here, to run repeatedly:

  if (missed > 0) {
    Serial.print("!!!Missed ");
    Serial.print(missed);
    Serial.println(" read cycles!!!");
    missed = 0;
  }

  readMicServerResponse();

  Serial.print("Average sound level is ");
  Serial.println(bufferAverage());
  Serial.print("RMS of sound level is ");
  Serial.println(bufferRMS());

#if defined(USE_DHT)
  Serial.print("DHT Temp: ");
  Serial.print(dht.readTemperature());
  Serial.print("C Humidity: ");
  Serial.print(dht.readHumidity());
  Serial.println("%");
#endif

#if defined(USE_RTC)
  Serial.print("Temperature is ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");
#endif

  Serial.print("At ");
  time_t t = epochTime; // Copying because you can't cast from a volatile to a const
  Serial.println(asctime(gmtime(&t)));

  if (errorFlag && epochTime % 2) {
    digitalWrite(ERROR_LED_PIN, HIGH);
  } else {
    digitalWrite(ERROR_LED_PIN, LOW);
  }

  if (needReinit) registerMicServer();
}

#if defined(USE_RTC)
void initializeRtcFromNtp() {
  while (!timeClient.update()) {
    delay(500);
    timeClient.forceUpdate();
  }
  epochTime = timeClient.getEpochTime();
  unsigned long epochMicros = timeClient.getEpochMicros();
  // Set up ticker so it rolls over at the top of the second.
  // Do this before we call rtc.adjust(...) because that could take a while.
  ticker = (uint16_t) (epochMicros / microsPerClockCycle);

  const time_t t = epochTime;
  struct tm *now = gmtime(&t);

  Serial.print("Initializing RTC with time ");
  Serial.print(asctime(now));
  rtc.adjust(DateTime(now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec));
}
#endif

float bufferAverage() {
  float sum = 0.0;
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    sum += read_buf[i];
  }
  return sum / READ_BUF_SIZE;
}

float bufferRMS() {
  float avg = (float) bufferAverage();
  float sum_sq = 0.0;
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    sum_sq += sq(((float) read_buf[i])- avg);
  }
  return sqrt(sum_sq/READ_BUF_SIZE);
}

void sendBuffer() {
  int rc;
  rc = micServerUDP.beginPacket(micServerIP, MIC_UDP_PORT);
  if (rc == 0) {
    LOG_ERROR("UNABLE TO ALLOCATE PACKET for sendBuffer()");
    return;
  }
  if (micServerUDP.write('M') != 1) LOG_ERROR("Unable to write cmd");
  if (micServerUDP.write(MIC_ID, 3) != 3) LOG_ERROR("Unable to write id");
  if (writeTimeInfo(micServerUDP, timeClient.getEpochTime(), timeClient.getEpochMicros()) != 8) LOG_ERROR("Unable to write time");
  if (writeTimeInfo(micServerUDP, read_bufStartEpoch, read_bufStartMicros) != 8) LOG_ERROR("Unable to write start_time");
  uint16_t microsPerCycle = (uint16_t) microsPerClockCycle;
  if (micServerUDP.write((const uint8_t*) &microsPerCycle, sizeof(microsPerCycle)) != sizeof(microsPerCycle)) LOG_ERROR("Unable to write cycle micros");
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    if (micServerUDP.write((const uint8_t*) &read_buf[i], sizeof(uint16_t)) != sizeof(uint16_t)) LOG_ERROR("Unable to write data");
  }
  if (micServerUDP.endPacket() == 0) LOG_ERROR("Unable to send measurement packet");

#if defined(ESP8266)
  yield(); // give the chip time to send the packet
#endif

  // Now reset the buffer times
  read_bufStartEpoch = timeClient.getEpochTime();
  read_bufStartMicros = timeClient.getEpochMicros();

  readMicServerResponse();
}

size_t writeTimeInfo(WiFiUDP &udp, uint32_t epoch, uint32_t epoch_micros) {
  size_t bytes_written = 0;
  bytes_written += udp.write((const uint8_t *) &epoch, sizeof(epoch));
  bytes_written += udp.write((const uint8_t *) &epoch_micros, sizeof(epoch_micros));
  return bytes_written;
}

void readMicServerResponse() {
  if (micServerUDP.parsePacket()) {
    unsigned char response[3];
    if (micServerUDP.read(response, 2) != 2) {
      LOG_ERROR("Unable to read response packet");
      return;
    }
    response[2] = 0; // Null terminate the string for parsing
    if (response[0] == 'E' && response[1] == 'R') {
      LOG_ERROR("Got Error code from server");
    } else if (response[0] == 'R' && response[1] == 'I') {
      needReinit = true;
    } else if (response[0] == 'O' && response[1] == 'K') {
      // don't need to do anything for okay
    } else {
      LOG_ERROR("Got unknown response code from server");
      Serial.println((const char *) response);
    }
  }
}

void registerMicServer() {
  int rc;
  rc = micServerUDP.beginPacket(micServerIP, MIC_UDP_PORT);
  if (rc == 0) {
    LOG_ERROR("UNABLE TO ALLOCATE PACKET for sendBuffer()");
    return;
  }
  if (micServerUDP.write('R') != 1) LOG_ERROR("Unable to write cmd");
  if (micServerUDP.write(MIC_ID, 3) != 3) LOG_ERROR("Unable to write id");
  if (writeTimeInfo(micServerUDP, timeClient.getEpochTime(), timeClient.getEpochMicros()) != 8) LOG_ERROR("Unable to write time");
  uint16_t temp;
#if defined(USE_DHT)
  temp = (uint16_t) ((dht.readTemperature() + 273.15)*10);
#elif defined(USE_RTC)
  temp = (uint16_t) ((rtc.getTemperature() + 273.15)*10);
#else 
  temp = 2981; // 298.1 K is 25 C.. which is a reasonable default
#endif
  if (micServerUDP.write((const uint8_t *) &temp, sizeof(temp)) != sizeof(temp)) LOG_ERROR("Unable to write temp"); 
  if (micServerUDP.endPacket() == 0) LOG_ERROR("Unable to send packet");
  
#if defined(ESP8266)
  yield(); // give the chip time to send the packet
#endif

  needReinit = false;
}

void setClockFromNTP() {
  timeClient.update() || Serial.println('Failed to update NTP client');
  epochTime = timeClient.getEpochTime();
  epochMicros = timeClient.getEpochMicros();
  ticker = (uint16_t) epochMicros / microsPerClockCycle;
  Serial.print("It's ");
  Serial.print(epochMicros);
  Serial.print(" microseconds past epoch time of ");
  Serial.println((unsigned long) epochTime);
}
