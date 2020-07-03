#if defined(ESP8266)
# include <ESP8266TimerInterrupt.h>
# include <ESP8266_ISR_Timer.h>
# include <ESP8266_ISR_Timer-Impl.h>

# include <DHT.h>
# include <DHT_U.h>

# define USE_DHT 1
# include <ESP8266WiFi.h>
#else // assume Feather M0 WiFi
# include <SPI.h>
# include <WiFi101.h>
# include "RTClib.h"
# define USE_RTC 1
#endif

#include <WiFiUdp.h>
#include <BetterNTP.h>
#include <time.h>

#include "wifi_creds.h"
const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASS;

const char ntpServer[] = "192.168.86.8";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer);
volatile time_t epochTime = 0;
volatile uint32_t epochMicros = 0;

#if defined(ESP8266)
 // Init ESP8266 timer 0
 ESP8266Timer ITimer;
# define CLOCK_RATE 10000
#else
 const uint8_t clockPin = 10; // the pin receiving the clockSignal.
# define CLOCK_RATE 8192
RTC_DS3231 rtc;
#endif

const float microsPerClockCycle = 1000000.0/CLOCK_RATE;
volatile uint8_t ticker_ticked = 0;  // increases 1 per clock cycle, needs to be reset manually
volatile uint16_t ticker = 0;  // increases 1 per clock cycle, rolls over every second.
volatile bool ticker_rollover = false;  // has the second rollover happened? reset manually

#if defined(ESP8266)
# define MICPIN A0
#else
# define MICPIN A2
#endif

#define READ_BUF_SIZE 200
uint8_t missed = 0;  // count of how many readings got skipped.
uint16_t read_buf[READ_BUF_SIZE];
uint8_t read_buf_index = 0;

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

void setup() {
  Serial.begin(115200);

#if defined(USE_DHT)
  Serial.println("Initializing temp sensor");
  dht.begin();
#endif

#if !defined(ESP8266)
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
#endif

  Serial.print("Initializing WIFI to talk to SSID ");
  Serial.print(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("!");

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
  if (rtc.lostPower()) {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    initializeRtcFromNtp();
  }
 
  pinMode(clockPin, INPUT_PULLUP);
  rtc.writeSqwPinMode(DS3231_SquareWave8kHz);
  attachInterrupt(digitalPinToInterrupt(clockPin), clock_tick, FALLING);
#endif

  setClockFromNTP();
}

void loop() {
  while (!ticker_rollover) {
    while (ticker_ticked == 0) delayMicroseconds(10);
    if (ticker_ticked > 1) missed += ticker_ticked - 1;
    ticker_ticked = 0; // reset ticker counter;
    // take measurement
    uint16_t reading = analogRead(MICPIN);
    read_buf[read_buf_index] = reading;
    read_buf_index = (read_buf_index + 1) % READ_BUF_SIZE;
  }
  ticker_rollover = false; // reset rollover flag
  // put your main code here, to run repeatedly:

  if (missed > 0) {
    Serial.print("!!!Missed ");
    Serial.print(missed);
    Serial.println(" read cycles!!!");
    missed = 0;
  }

  Serial.print("Average sound level is ");
  Serial.println(bufferAverage());
  
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
  ticker = (uint16_t) epochMicros/microsPerClockCycle;
  
  const time_t t = epochTime;
  struct tm *now = gmtime(&t);

  rtc.adjust(DateTime(now->tm_year+1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec));
}
#endif

uint16_t bufferAverage() {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    sum += read_buf[i]; 
  }
  return (uint16_t) (sum / READ_BUF_SIZE);
}

void setClockFromNTP() {
  timeClient.update() || Serial.println('Failed to update NTP client');
  epochTime = timeClient.getEpochTime();
  epochMicros = timeClient.getEpochMicros();
  ticker = (uint16_t) epochMicros/microsPerClockCycle;
  Serial.print("It's ");
  Serial.print(epochMicros);
  Serial.print(" microseconds past epoch time of ");
  Serial.println((unsigned long) epochTime);
}
