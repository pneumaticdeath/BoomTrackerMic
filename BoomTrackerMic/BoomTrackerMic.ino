//#define SERIAL_DEBUG
#if defined(ESP8266)
# define MIC_ID "002"
#elif defined(ESP32)
# define MIC_ID "003"
#else
# define MIC_ID "001"
//# define MIC_ID "004"
#endif

#if defined(ESP8266)
# include <ESP8266TimerInterrupt.h>
# include <ESP8266_ISR_Timer.h>
# include <ESP8266_ISR_Timer-Impl.h>

#define ERROR_LED_PIN 0
# define USE_DHT 1
# define DHT_PIN 13
# include <ESP8266WiFi.h>


#elif defined(ESP32)
//???
#include <WiFi.h>
#include <AsyncUDP.h>

#include <ESP32TimerInterrupt.h>

#else // assume Feather M0 WiFi

# include <SPI.h>
# include <WiFi101.h>
//# include "RTClib.h"
//# define USE_RTC 1
# define USE_DHT 1
# define DHT_PIN 11
# define USE_M0_INTERNAL_TIMER
# define ERROR_LED_PIN 13

# if defined(USE_M0_INTERNAL_TIMER)
/*
   Taken from https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
*/

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();
# endif
#endif

#if defined(USE_DHT)
# include <DHT.h>
# include <DHT_U.h>
DHT dht(DHT_PIN, DHT11);
#endif

#include <WiFiUdp.h>
#include "BetterNTP.h"
#include <time.h>
#include "wifi_creds.h"
const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASS;

//IPAddress ntpServer = IPAddress(192, 168, 86, 9);  // DON'T DO THIS... it causes NTPClient to do very strange things
const char *ntpServer = "192.168.86.9";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer);
volatile time_t epochTime = 0;
volatile uint32_t epochMicros = 0;

#if defined(ESP8266)
// Init ESP8266 timer 0
ESP8266Timer ITimer;
# define CLOCK_RATE 400
#elif defined(ESP32)
// TODO
#elif defined(USE_M0_INTERNAL_TIMER)
# define CLOCK_RATE 10000
#else
const uint8_t clockPin = 10; // the pin receiving the clock signal.
# define CLOCK_RATE 8192
RTC_DS3231 rtc;
#endif

const float microsPerClockCycle = 1000000.0 / CLOCK_RATE;
volatile uint8_t ticker_ticked = 0;  // increases 1 per clock cycle, needs to be reset manually
volatile uint16_t ticker = 0;  // increases 1 per clock cycle, rolls over every second.
volatile bool ticker_rollover = false;  // has the second rollover happened? reset manually
uint16_t updateCounter = 0;

#if defined(ESP8266)
# define MICPIN A0
#else
# define MICPIN A2
#endif


#define NUM_BUFFERS 4
#define READ_BUF_SIZE 512
volatile bool start_reading = false;
volatile uint16_t read_bufs[NUM_BUFFERS][READ_BUF_SIZE];
volatile uint16_t which_buf = 0;
volatile uint16_t read_buf_index = 0;
volatile uint32_t read_bufStartEpoch[NUM_BUFFERS];
volatile uint32_t read_bufStartMicros[NUM_BUFFERS];
volatile uint16_t read_bufMicrosPerCycle[NUM_BUFFERS];
volatile uint16_t reading = 0;
volatile uint16_t missed = 0;


IPAddress micServerIP = IPAddress(192, 168, 86, 9);
#define MIC_UDP_PORT 19086
WiFiUDP micServerUDP;
bool needReinit = true;
#if defined(ESP8266)
#define TRIGGER_RATIO 20
float running_avg = 350.0; // 1024 full scale, and average voltage from mic is 0.35v out of 1v
float trigger_threshold = 1.5 * TRIGGER_RATIO;
#else
#define TRIGGER_RATIO 25
float running_avg = 2048.0; //4096 full scale and average mic voltage is 1.65v out of 3.3v
float trigger_threshold = 10.0 * TRIGGER_RATIO;
#endif
volatile bool send_now = false;
volatile bool send_buffer_flag[NUM_BUFFERS];
uint16_t buffers_sent = 0;


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
  if (start_reading) {
    reading = analogRead(MICPIN);
    read_bufs[which_buf][read_buf_index] = reading;
    if (reading > 0) {
      if (abs(reading - running_avg) > trigger_threshold) {
        send_buffer_flag[which_buf] = true;
      }
      reading = 0;
    } else {
      missed++;
    }
    read_buf_index = (read_buf_index + 1) % READ_BUF_SIZE;
    if (read_buf_index == 0) {
      uint32_t newEpoch = timeClient.getEpochTime();
      uint32_t newMicros = timeClient.getEpochMicros();
      uint32_t deltaMicros = (newEpoch-read_bufStartEpoch[which_buf])*1000000 + (newMicros-read_bufStartMicros[which_buf]);
      //Serial.printf("deltaMicros = %ul\n",deltaMicros);
      read_bufMicrosPerCycle[which_buf] = deltaMicros/READ_BUF_SIZE;
      //Serial.printf("PerCycle = %ud\n",read_bufMicrosPerCycle[which_buf]);
      send_now = true;
      which_buf = (which_buf + 1) % NUM_BUFFERS;
      // Reset the buffer times
      read_bufStartEpoch[which_buf] = newEpoch;
      read_bufStartMicros[which_buf] = newMicros;
    }
  }
}

bool errorFlag = false;
uint32_t errorTimeout = 0;

void LOG_ERROR(const char *message) {
  String errmsg = "ERROR ";
  errmsg += message;
  LOG(errmsg.c_str());
  errorFlag = true;
  if (errorTimeout < 86400) {
    errorTimeout += 600; // 10 more minutes of flashing
  }
}

void LOG(const char *message) {
#if defined(SERIAL_DEBUG)
  Serial.println(message);
#endif
  if (micServerUDP.beginPacket(micServerIP, MIC_UDP_PORT) == 0) {
    Serial.println("Unable to start log packet");
    return;
  }
  micServerUDP.write('L');
  micServerUDP.write(MIC_ID, (size_t) 3);
  writeTimeInfo(micServerUDP, timeClient.getEpochTime(), timeClient.getEpochMicros());
  size_t len = strlen(message);
  micServerUDP.write(message, len);
  if (micServerUDP.endPacket() == 0) Serial.println("Unable to send log message");
#if defined(ESP8266)
  yield();
#endif
}

void setup() {
  Serial.begin(115200);

#if defined(SERIAL_DEBUG)
  while (!Serial);
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

#if defined(USE_DHT)
  LOG("Initializing temp sensor");
  dht.begin();
#endif


  // Start NTP client
  timeClient.begin();
  timeClient.setUpdateInterval(1000); // 1 second

#if !defined(ESP8266)
  analogReadResolution(12);
#endif

  pinMode(ERROR_LED_PIN, OUTPUT);

  setClockFromNTP();

#if defined(USE_RTC)
  if (! rtc.begin()) {
    LOG("Can't find RTC");
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

#if defined(USE_M0_INTERNAL_TIMER)
  startTimer(CLOCK_RATE);
#endif

#if defined(ESP8266)
  if (ITimer.attachInterruptInterval((uint32_t) microsPerClockCycle, clock_tick))
    LOG((String("Starting  ITimer OK, millis() = ") + String(millis())).c_str());
  else
    LOG("Can't set ITimer. Select another freq. or interval");
#endif

  micServerUDP.begin(MIC_UDP_PORT);
  registerMicServer();
  LOG("Starting up");
  // First buffer doesn't have it's start time set yet.
  read_bufStartEpoch[which_buf] = timeClient.getEpochTime();
  read_bufStartMicros[which_buf] = timeClient.getEpochMicros();
  start_reading = true;
}

void loop() {
  String msg;
  while (!ticker_rollover) {
    while (ticker_ticked == 0) {
      delayMicroseconds(10);
    }
    ticker_ticked = 0;
    if (send_now) {
      sendBuffer();
      send_now = false;
    }
  }
  ticker_rollover = false; // reset rollover flag
  // put your main code here, to run repeatedly:

  if (++updateCounter >= 5) {
    updateCounter = 0;
    refreshNTP();
  }

  if (buffers_sent > 0) {
    msg = String(buffers_sent);
    msg += " buffers sent";
    LOG(msg.c_str());
    buffers_sent = 0;
  }

  readMicServerResponse();

  if ( missed > 0 ) {
    msg = "Missed ";
    msg += missed;
    msg += " readings (";
    msg += 100 * missed / CLOCK_RATE;
    msg += "%)";
    LOG(msg.c_str());
    missed = 0;
  }

  msg = "Running average sound level is ";
  msg += running_avg;
  LOG(msg.c_str());
  msg = "RMS of sound level is ";
  msg += bufferRMS(which_buf);
  LOG(msg.c_str());
  msg = "Trigger threshold = ";
  msg += trigger_threshold;
  LOG(msg.c_str());

#if defined(USE_DHT)
  msg = "DHT Temp: ";
  msg += dht.readTemperature();
  msg += "C Humidity: ";
  msg += dht.readHumidity();
  msg += "%";
  LOG(msg.c_str());
#endif

#if defined(USE_RTC)
  msg = "Temperature is ";
  msg += rtc.getTemperature();
  msg += " C";
  LOG(msg.c_str());
#endif

#ifdef SERIAL_DEBUG
  Serial.print("At ");
  time_t t = timeClient.getEpochTime();
  Serial.println(asctime(gmtime(&t)));
#endif

  if (errorFlag && epochTime % 2) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    if (errorTimeout > 0) {
      errorTimeout -= 2;
    } else {
      errorFlag = false;
    }
  } else {
    digitalWrite(ERROR_LED_PIN, LOW);
  }

  if (needReinit) registerMicServer();
}

#if defined(USE_RTC)
void initializeRtcFromNtp() {
  timeClient.update();

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

float bufferAverage(uint8_t bufNum) {
  float sum = 0.0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    if (read_bufs[bufNum][i] > 0) { // don't average missing values
      sum += read_bufs[bufNum][i];
      count++;
    }
  }
  if (count > 0) return sum / count;
  else return -1.0;
}

float bufferRMS(uint8_t bufNum) {
  float sum_sq = 0.0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
    if (read_bufs[bufNum][i] > 0) {
      sum_sq += sq(((float) read_bufs[bufNum][i]) - running_avg);
      count++;
    }
  }
  if (count > 0) return sqrt(sum_sq / count);
  else return -1.0;
}

void sendBuffer() {
  int rc;
  String msg_status;

  for (uint16_t old_buf = 0; old_buf < NUM_BUFFERS; old_buf++ ) {
    if (old_buf != which_buf && send_buffer_flag[old_buf]) {
      send_buffer_flag[old_buf] = false;
      buffers_sent++;

      rc = micServerUDP.beginPacket(micServerIP, MIC_UDP_PORT);
      if (rc == 0) {
        msg_status += "UNABLE TO ALLOCATE PACKET for sendBuffer()! ";
      } else {
        if (micServerUDP.write('M') != 1) msg_status += "Unable to write cmd! ";
        if (micServerUDP.write(MIC_ID, 3) != 3) msg_status += "Unable to write id! ";
        if (writeTimeInfo(micServerUDP, timeClient.getEpochTime(), timeClient.getEpochMicros()) != 8) msg_status += "Unable to write time! ";
        uint32_t startEpoch = read_bufStartEpoch[old_buf];
        uint32_t startMicros = read_bufStartMicros[old_buf];
        if (writeTimeInfo(micServerUDP, startEpoch, startMicros) != 8) msg_status += "Unable to write start_time! ";
        uint16_t microsPerCycle = read_bufMicrosPerCycle[old_buf];
        if (micServerUDP.write((const uint8_t*) &microsPerCycle, sizeof(microsPerCycle)) != sizeof(microsPerCycle)) msg_status += "Unable to write cycle micros! ";
        for (uint16_t i = 0; i < READ_BUF_SIZE; i++) {
          if (micServerUDP.write((const uint8_t*) &read_bufs[old_buf][i], sizeof(uint16_t)) != sizeof(uint16_t)) msg_status += "Unable to write data! ";
        }
        if (micServerUDP.endPacket() == 0) msg_status += "Unable to send measurement packet! ";
      }
#if defined(ESP8266)
      yield(); // give the chip time to send the packet
#endif
    }
  }

  uint16_t last_buf = (which_buf + NUM_BUFFERS - 1) % NUM_BUFFERS;
  running_avg = (9 * running_avg + bufferAverage(last_buf)) / 10.0;
  trigger_threshold = (29 * trigger_threshold + TRIGGER_RATIO * bufferRMS(last_buf)) / 30.0;
  readMicServerResponse();
  if (msg_status.length() != 0) LOG_ERROR(msg_status.c_str());
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
  temp = (uint16_t) ((dht.readTemperature() + 273.15) * 10);
#elif defined(USE_RTC)
  temp = (uint16_t) ((rtc.getTemperature() + 273.15) * 10);
#else
  temp = 2981; // 298.1 K is 25 C.. which is a reasonable default
#endif
  if (micServerUDP.write((const uint8_t *) &temp, sizeof(temp)) != sizeof(temp)) LOG_ERROR("Unable to write temp");
  if (micServerUDP.endPacket() == 0) LOG_ERROR("Unable to send packet");
#if defined(ESP8266)
  yield(); // give the chip time to send the packet
#endif

  LOG("Reinit finished");
  needReinit = false;
}

void setClockFromNTP() {
  if (!timeClient.update()) LOG_ERROR("Failed to update NTP client");
  epochTime = timeClient.getEpochTime();
  epochMicros = timeClient.getEpochMicros();
  ticker = (uint16_t) epochMicros / microsPerClockCycle;
  String msg = "Initializing clock.  It's ";
  msg += epochMicros;
  msg += " microseconds past epoch time of ";
  msg += (unsigned long) epochTime;
  LOG(msg.c_str());
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
    LOG(msg.c_str());

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
    LOG(msg.c_str());

    msg = "Last delay: ";
    msg += timeClient.getDelay();
    snprintf(microStr, 8, ".%06d", timeClient.getDelayMicros());
    microStr[7] = '\0';
    msg += microStr;
    msg += " seconds";
    LOG(msg.c_str());

  } else {
    LOG_ERROR("Time update failed");
  }
}

#if defined(USE_M0_INTERNAL_TIMER)

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we call the clock_tick() function
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    clock_tick();
  }
}
#endif
