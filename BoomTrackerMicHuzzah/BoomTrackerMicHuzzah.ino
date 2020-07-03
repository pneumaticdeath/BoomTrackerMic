#include <ESP8266TimerInterrupt.h>
#include <ESP8266_ISR_Timer.h>
#include <ESP8266_ISR_Timer-Impl.h>

#include <DHT.h>
#include <DHT_U.h>

#include <BetterNTP.h>
#include <time.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "wifi_creds.h"
const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASS;

const char ntpServer[] = "192.168.86.8";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer);
volatile time_t epochTime = 0;
volatile uint32_t epochMicros = 0;

// Init ESP8266 timer 0
ESP8266Timer ITimer;

#define CLOCK_RATE 10000
const float microsPerClockCycle = 1000000.0/CLOCK_RATE;
volatile uint8_t ticker_ticked = 0;  // increases 1 per clock cycle, needs to be reset manually
volatile uint16_t ticker = 0;  // increases 1 per clock cycle, rolls over every second.
volatile bool ticker_rollover = false;  // has the second rollover happened? reset manually

#define MICPIN A0
#define READ_BUF_SIZE 200
uint8_t missed = 0;  // count of how many readings got skipped.
uint16_t read_buf[READ_BUF_SIZE];
uint8_t read_buf_index = 0;

#define DHTPIN 5
DHT dht(DHTPIN, DHT11);

void ICACHE_RAM_ATTR clock_tick() {
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

  Serial.println("Initializing temp sensor");
  dht.begin();

  Serial.print("Initializing WIFI to talk to SSID ");
  Serial.print(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("!");

  if (ITimer.attachInterruptInterval((uint32_t) microsPerClockCycle, clock_tick))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or interval");

  setClockFromNTP();
}

uint16_t bufferAverage() {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    sum += read_buf[i]; 
  }
  return (uint16_t) (sum / READ_BUF_SIZE);
}

void setClockFromNTP() {
  while (!timeClient.update()) {
    delay(5);
    timeClient.forceUpdate();
  }
  epochTime = timeClient.getEpochTime();
  epochMicros = timeClient.getEpochMicros();
  ticker = (uint16_t) epochMicros/microsPerClockCycle;
  Serial.print("It's ");
  Serial.print(epochMicros);
  Serial.print(" microseconds past epoch time of ");
  Serial.println(epochTime);
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

  Serial.print("Temp: ");
  Serial.print(dht.readTemperature());
  Serial.print("C Humidity: ");
  Serial.print(dht.readHumidity());
  Serial.println("%");
}
