#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <BetterNTP.h>
#include "RTClib.h"
#include <time.h>

#include "wifi_creds.h"
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;
int status = WL_IDLE_STATUS;     // the WiFi radio's status

char ntpServer[] = "192.168.86.8";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer);

const uint8_t clockPin = 10; // the pin receiving the clockSignal.
#define CLOCK_RATE 8192
const float microsPerClockCycle = 1000000.0/CLOCK_RATE;
RTC_DS3231 rtc;

volatile time_t currentEpochTime;
volatile uint16_t ticker = 0;
volatile bool ticker_ticked = false;
volatile bool ticker_rollover = false;

void setup() {
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  
  Serial.begin(115200);

  while (!Serial);

  if (! rtc.begin()) {
    Serial.println("Can't find RTC");
    Serial.flush();
    abort();
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  Serial.print("Initializing WIFI to talk to SSID ");
  Serial.print(ssid);
  status = WiFi.begin(ssid, pass);
  while (status != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    status = WiFi.status();
  }
  Serial.println("!");

  // Start NTP client
  timeClient.begin();
  
  // If the RTC isn't initialized or has lost power, we need to initialize it.
  // Replace this with NTP code once we've got it working.
  if (rtc.lostPower()) {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    initializeRtcFromNtp();
  }
  
  pinMode(clockPin, INPUT_PULLUP);
  rtc.writeSqwPinMode(DS3231_SquareWave8kHz);
  attachInterrupt(digitalPinToInterrupt(clockPin), clock_tick, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!ticker_rollover) {
    while(!ticker_ticked) {
      delayMicroseconds(10);
    }
    ticker_ticked = false;
    // Do the sampling routine here.
    
  }
  // we only fall out of the loop when ticker_rollover is true, so reset it
  ticker_rollover = false;
  Serial.print("Temperature is ");
  Serial.print(rtc.getTemperature());
  Serial.print(" C at ");
  time_t epochtime = currentEpochTime;
  Serial.println(asctime(gmtime(&epochtime)));
}

void initializeRtcFromNtp() {
  while (!timeClient.update()) {
    delay(500);
    timeClient.forceUpdate();
  }
  currentEpochTime = timeClient.getEpochTime();
  unsigned long currentEpochMicros = timeClient.getEpochMicros();
  // Set up ticker so it rolls over at the top of the second.
  // Do this before we call rtc.adjust(...) because that will take a while.
  ticker = (uint16_t) currentEpochMicros/microsPerClockCycle;
  
  const time_t t = currentEpochTime;
  struct tm *now = gmtime(&t);

  rtc.adjust(DateTime(now->tm_year+1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec));
}

// ISR for clock signal
void clock_tick() {
  ticker++;
  ticker_ticked = true;
  if (ticker >= CLOCK_RATE) {
    ticker=0;
    currentEpochTime++;
    ticker_rollover = true;
  }
}
