/**
 * The MIT License (MIT)
 * Copyright (c) 2015 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "BetterNTP.h"

//#define DEBUG_NTPClient

String tmpStr;
const char *tmToStr(unsigned long secs, unsigned long microseconds, bool neg) {
  tmpStr = "";
  if (neg) tmpStr += "-";
  tmpStr += secs;
  char buf[8];
  snprintf(buf, 8, ".%06d", microseconds);
  tmpStr += buf;
  return tmpStr.c_str();
}
// We need to do math on values of seconds combined with microseconds.
// offset = ((t1-t0) + (t2-t3))/2
// delay = (t3-t0) - (t2-t1)  -- this will always be positive.  If it comes out negative, we've got some very weird stuff going on.

// This math gets hairy.  We can't assume the result will fit into an signed long, so we have to keep it unsigned, but we need to detect whether the result is negative
// effetively making this 1's complement arithmatic
void subtractTimes(unsigned long aSecs, unsigned long aMicros, bool aNeg, unsigned long bSecs, unsigned long bMicros, bool bNeg, unsigned long *diffSecs, unsigned long *diffMicros, bool *diffNeg) {
  if (!aNeg && bNeg) { // if a >= 0 and b < 0 then a-b = a+|b|
    addTimes(aSecs, aMicros, false, bSecs, bMicros, false, diffSecs, diffMicros, diffNeg);
  } else if (aNeg && bNeg) { // if a < 0 and b < 0 then a-b = -1*(|a|-|b|)
    subtractTimes(aSecs, aMicros, false, bSecs, bMicros, false, diffSecs, diffMicros, diffNeg);
    *diffNeg = !(*diffNeg);
  } else if (aNeg && !bNeg) { // if a < 0 and b >= 0 then a-b = -1*(|a|+ b)
    addTimes(aSecs, aMicros, false, bSecs, bMicros, false, diffSecs, diffMicros, diffNeg);
    *diffNeg = true;
  } else { // both a and b non-negative
    bool borrow = false;
    if (bMicros > aMicros) {
      borrow = true; // borrow from whole seconds;
      *diffMicros = 1000000 + aMicros - bMicros;
    } else {
      borrow = false;
      *diffMicros = aMicros - bMicros;
    }
    if ( (bSecs + (borrow ? 1 : 0)) > aSecs) {
      *diffNeg = true;
      *diffSecs = (bSecs + (borrow ? 1 : 0)) - aSecs;
    } else {
      *diffNeg = false;
      *diffSecs =  aSecs - (bSecs + (borrow ? 1 : 0));
    }
    if (*diffNeg && *diffMicros != 0) { // if we end up with a negative result, then we need to flip the sign of the microseconds part
      *diffMicros = 1000000 - *diffMicros;
      *diffSecs -= 1;
    }
  }
}

void addTimes(unsigned long aSecs, unsigned long aMicros, bool aNeg, unsigned long bSecs, unsigned long bMicros, bool bNeg, unsigned long * sumSecs, unsigned long * sumMicros, bool * sumNeg) {
  if (aNeg and !bNeg) {
    // Since if a<0 and b>=0, a+b = b-|a|
    subtractTimes(bSecs, bMicros, false,  aSecs, aMicros, false, sumSecs, sumMicros, sumNeg);
  } else if (aNeg && bNeg) { // if a < 0 and b < 0 then a+b = -1*(|a|+|b|)
    addTimes(aSecs, aMicros, false, bSecs, bMicros, false, sumSecs, sumMicros, sumNeg);
    *sumNeg = true;
  } else if (!aNeg && bNeg) { // if a>=0 and b<0 then a+b = a-|b|
    subtractTimes(aSecs, aMicros, false, bSecs, bMicros, false, sumSecs, sumMicros, sumNeg);
  } else {
    *sumNeg = false;
    *sumMicros = aMicros + bMicros;
    if (*sumMicros >= 1000000) {
      *sumMicros -= 1000000;
      *sumSecs = aSecs + bSecs + 1;
    } else {
      *sumSecs = aSecs + bSecs;
    }
  }
}

void divBy2(unsigned long * secs, unsigned long * microsecs) {
  if ((*secs % 2) == 1) {
    *microsecs = 500000 + (*microsecs / 2);
  } else {
    *microsecs /= 2;
  }
  *secs = *secs / 2;
}

NTPClient::NTPClient(UDP& udp) {
  this->_udp            = &udp;
}

NTPClient::NTPClient(UDP& udp, int timeOffset) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
}

NTPClient::NTPClient(UDP& udp, const char* poolServerName) {
  this->_udp            = &udp;
  this->_poolServerName = poolServerName;
}

NTPClient::NTPClient(UDP& udp, const char* poolServerName, int timeOffset) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
}

NTPClient::NTPClient(UDP& udp, const char* poolServerName, int timeOffset, unsigned long updateInterval) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
  this->_updateInterval = updateInterval;
}

void NTPClient::begin() {
  this->begin(NTP_DEFAULT_LOCAL_PORT);
}

void NTPClient::begin(int port) {
  this->_port = port;

  this->_udp->begin(this->_port);

  this->_udpSetup = true;
}

bool NTPClient::isValid(byte * ntpPacket)
{
  //Perform a few validity checks on the packet
  if ((ntpPacket[0] & 0b11000000) == 0b11000000)		//Check for LI=UNSYNC
    return false;

  if ((ntpPacket[0] & 0b00111000) >> 3 < 0b100)		//Check for Version >= 4
    return false;

  if ((ntpPacket[0] & 0b00000111) != 0b100)			//Check for Mode == Server
    return false;

  if ((ntpPacket[1] < 1) || (ntpPacket[1] > 15))		//Check for valid Stratum
    return false;

  if (	ntpPacket[16] == 0 && ntpPacket[17] == 0 &&
        ntpPacket[18] == 0 && ntpPacket[19] == 0 &&
        ntpPacket[20] == 0 && ntpPacket[21] == 0 &&
        ntpPacket[22] == 0 && ntpPacket[22] == 0)		//Check for ReferenceTimestamp != 0
    return false;

  return true;
}

bool NTPClient::forceUpdate() {
#ifdef DEBUG_NTPClient
  Serial.println("Update from NTP Server");
#endif
  // flush any existing packets
  while(this->_udp->parsePacket() != 0)
    this->_udp->flush();
  this->sendNTPPacket();

  // Wait till data is there or timeout...
  unsigned long timeout = 0;
  int cb = 0;
  do {
    delayMicroseconds ( 10 );
    cb = this->_udp->parsePacket();

    if(cb > 0)
    {
      this->_udp->read(this->_packetBuffer, NTP_PACKET_SIZE);
      if (!this->isValid(this->_packetBuffer)) {
#ifdef DEBUG_NTPCLient
        Serial.println("Packet failed validity test");
#endif
        cb = 0;

      }
    }

    if(timeout > 100000) {
#ifdef DEBUG_NTPClient
      Serial.println("Timeout waiting for packet");
#endif
      return false; // timeout after 1 second
    }
    timeout++;
  } while (cb == 0);

  // Do this first to avoid delay;
  unsigned long t3Secs, t3Micros;
  if (this->_currentEpoc != 0) {
    t3Secs = this->getEpochTime() + SEVENZYYEARS;
    t3Micros = this->getEpochMicros();
  } else {
    t3Secs = 0;
    t3Micros = micros();
  }

  unsigned long previousUpdate = this->_lastUpdate;
  this->_lastUpdate = micros();
  unsigned long deltaMicrosMeasured;
  if (previousUpdate > this->_lastUpdate) {
    // we've hit rollover
    deltaMicrosMeasured = this->_lastUpdate + 1 + (4294967295UL - previousUpdate);
  } else {
    deltaMicrosMeasured = this->_lastUpdate - previousUpdate;
  }

  unsigned long t0Secs, t0Micros;
  unsigned long t1Secs, t1Micros;
  unsigned long t2Secs, t2Micros;

  this->extractTimestampAt(24, t0Secs, t0Micros);
  this->extractTimestampAt(32, t1Secs, t1Micros);
  this->extractTimestampAt(40, t2Secs, t2Micros);

  if (t0Secs == 0 && t0Micros == 0) {
#ifdef DEBUG_NTPClient
    Serial.println("Got t0=0 from server, reverting to saved");
#endif
    t0Secs = this->_sentSecs;
    t0Micros = this->_sentMicros;
  }

#ifdef DEBUG_NTPClient
  Serial.print("T0 = ");
  Serial.println(tmToStr(t0Secs, t0Micros, false));
  Serial.print("T1 = ");
  Serial.println(tmToStr(t1Secs, t1Micros, false));
  Serial.print("T2 = ");
  Serial.println(tmToStr(t2Secs, t2Micros, false));
  Serial.print("T3 = ");
  Serial.println(tmToStr(t3Secs, t3Micros, false));
#endif

  unsigned long t1MinusT0Secs, t1MinusT0Micros;
  bool t1MinusT0Neg;
  unsigned long t2MinusT3Secs, t2MinusT3Micros;
  bool t2MinusT3Neg;
  subtractTimes(t1Secs, t1Micros, false, t0Secs, t0Micros, false, &t1MinusT0Secs, &t1MinusT0Micros, &t1MinusT0Neg);
  subtractTimes(t2Secs, t2Micros, false, t3Secs, t3Micros, false, &t2MinusT3Secs, &t2MinusT3Micros, &t2MinusT3Neg);
  divBy2(&t1MinusT0Secs, &t1MinusT0Micros);
  divBy2(&t2MinusT3Secs, &t2MinusT3Micros);
  unsigned long offsetSecs, offsetMicros;
  bool offsetNeg;
  addTimes(t1MinusT0Secs, t1MinusT0Micros, t1MinusT0Neg, t2MinusT3Secs, t2MinusT3Micros, t2MinusT3Neg, &offsetSecs, &offsetMicros, &offsetNeg);
  //divBy2(&offsetSecs, &offsetMicros);

#ifdef DEBUG_NTPClient
  Serial.print("(t1-t0)/2 = ");
  Serial.println(tmToStr(t1MinusT0Secs, t1MinusT0Micros, t1MinusT0Neg));
  Serial.print("(t2-t3)/2 = ");
  Serial.println(tmToStr(t2MinusT3Secs, t2MinusT3Micros, t2MinusT3Neg));
  Serial.print("offset = ");
  Serial.println(tmToStr(offsetSecs, offsetMicros, offsetNeg));
#endif

  this->_offsetSecs = offsetSecs;
  this->_offsetMicros = offsetMicros;
  this->_offsetNeg = offsetNeg;

  unsigned long t3MinusT0Secs, t3MinusT0Micros;
  bool t3MinusT0Neg;
  unsigned long t2MinusT1Secs, t2MinusT1Micros;
  bool t2MinusT1Neg;
  unsigned long delaySecs, delayMicros;
  bool delayNeg; // should never be true, but we need something to pass in
  subtractTimes(t3Secs, t3Micros, false, t0Secs, t0Micros, false, &t3MinusT0Secs, &t3MinusT0Micros, &t3MinusT0Neg);
  subtractTimes(t2Secs, t2Micros, false, t1Secs, t1Micros, false, &t2MinusT1Secs, &t2MinusT1Micros, &t2MinusT1Neg);
  subtractTimes(t3MinusT0Secs, t3MinusT0Micros, t3MinusT0Neg, t2MinusT1Secs, t2MinusT1Micros, t2MinusT1Neg, &delaySecs, &delayMicros, &delayNeg);

#ifdef DEBUG_NTPClient
  Serial.print("t3 - t0 = ");
  Serial.println(tmToStr(t3MinusT0Secs, t3MinusT0Micros, t3MinusT0Neg));
  Serial.print("t2 - t1 = ");
  Serial.println(tmToStr(t2MinusT1Secs, t2MinusT1Micros, t2MinusT1Neg));
  Serial.print("delay = ");
  Serial.println(tmToStr(delaySecs, delayMicros, delayNeg));
#endif

  if (delayNeg) {
    Serial.println("Got negative delay in NTPClient.. not possible");
  }
  this->_delaySecs = delaySecs;
  this->_delayMicros = delayMicros;

  if (offsetSecs > 1000) { // for offsets more then 1000 seconds we just use the server time
    this->_currentEpoc = t2Secs - SEVENZYYEARS;
    this->_currentMicros = t2Micros;
#ifdef DEBUG_NTPClient
    Serial.println("Offset huge, defaulting to server time");
#endif
  } else if (offsetSecs > 0 || offsetMicros >= 500000) { // for corrections of more than half a second, we just jump
    this->_currentEpoc = t3Secs + ((offsetNeg ? -1 : 1) * offsetSecs) - SEVENZYYEARS;
    if (offsetNeg && offsetMicros > t3Micros) {
      this->_currentMicros = 1000000 + t3Micros - offsetSecs;
      this->_currentEpoc--; // borrow
    } else {
      this->_currentMicros = t3Micros + ((offsetNeg ? -1 : 1) * offsetMicros);
      while (this->_currentMicros >= 1000000) {
        this->_currentMicros -= 1000000;
        this->_currentEpoc++; // carry
      }
    }
#ifdef DEBUG_NTPClient
    Serial.println("Offset too large, so jumping");
#endif
  } else {
    // assume offsetSecs is zero in this section.
    this->_currentEpoc = t3Secs - SEVENZYYEARS;
    this->_currentMicros = t3Micros;
    this->_skew = (offsetNeg ? -1 : 1) * offsetMicros;
    // What I actually want in the next line is correction = (4*correction + skew/deltaSeconds)/5
    // so that new correction is 4/5 the old correction and 1/5 the skew spread out over the number of seconds
    // so that the correction will be somewhat stable, but will trend toward eliminationg the skew.
    signed long spreadSkew = this->_skew / ((signed long) deltaMicrosMeasured / 1000000);
    this->_correction = (4 * this->_correction + spreadSkew) / 5 ;
#ifdef DEBUG_NTPClient
    Serial.print("skew = ");
    Serial.print(this->_skew);
    Serial.println(" microseconds");
    Serial.print("spreadSkew = ");
    Serial.print(spreadSkew);
    Serial.println(" microseconds/second");
    Serial.print("correction factor = ");
    Serial.print(this->_correction);
    Serial.println(" microseconds/second");
#endif
  }
  return true;
}

void NTPClient::extractTimestampAt(int at, unsigned long & secs, unsigned long & microseconds) {
  unsigned long highWord = word(this->_packetBuffer[at], this->_packetBuffer[at + 1]);
  unsigned long lowWord = word(this->_packetBuffer[at + 2], this->_packetBuffer[at + 3]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  secs = highWord << 16 | lowWord;

  unsigned long fracHighWord = word(this->_packetBuffer[at + 4], this->_packetBuffer[at + 5]);
  unsigned long fracLowWord = word(this->_packetBuffer[at + 6], this->_packetBuffer[at + 7]);
  microseconds = (unsigned long) ((fracHighWord << 16 | fracLowWord) / 4294.967296);

}
bool NTPClient::update() {
  unsigned long nowMicros = micros();
  if (nowMicros <= this->_lastUpdate                                   // Update if we hit a rollover
      || (nowMicros - this->_lastUpdate >= this->_updateInterval * 1000) // Update after _updateInterval
      || this->_lastUpdate == 0) {                                       // Update if there was no update yet.
    if (!this->_udpSetup) this->begin();                               // setup the UDP client if needed
    return this->forceUpdate();
  }
  return true;
}

unsigned long NTPClient::getEpochTime() {
  unsigned long nowMicros = micros();
  unsigned long deltaMicros = 0;
  if (nowMicros < this->_lastUpdate) {
    deltaMicros += nowMicros + 1 + (4294967295UL - this->_lastUpdate);
  } else {
    deltaMicros +=  nowMicros - this->_lastUpdate;
  }
  // Now we need to skew and/or correct
  if ( deltaMicros < 1000000 ) {
    deltaMicros += (signed long) (((float) this->_skew)*deltaMicros/1000000.0);
  } else {
    deltaMicros += this->_skew + (signed long) (((float) this->_correction)*(deltaMicros-1000000.0)/1000000.0);
  }
  deltaMicros += this->_currentMicros;
  this->_microsAtLastEpochFetch = deltaMicros % 1000000;
  unsigned long returnVal = this->_timeOffset + // User offset
                            this->_currentEpoc + // Epoch returned by the NTP server
                            (deltaMicros / 1000000UL); // Time since last update
  return returnVal;
}

unsigned long NTPClient::getEpochMicros() {
  return this->_microsAtLastEpochFetch;
}

int NTPClient::getDay() {
  return (((this->getEpochTime()  / 86400L) + 4 ) % 7); //0 is Sunday
}
int NTPClient::getHours() {
  return ((this->getEpochTime()  % 86400L) / 3600);
}
int NTPClient::getMinutes() {
  return ((this->getEpochTime() % 3600) / 60);
}
int NTPClient::getSeconds() {
  return (this->getEpochTime() % 60);
}

String NTPClient::getFormattedTime(unsigned long secs) {
  unsigned long rawTime = secs ? secs : this->getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return hoursStr + ":" + minuteStr + ":" + secondStr;
}

// Based on https://github.com/PaulStoffregen/Time/blob/master/Time.cpp
// currently assumes UTC timezone, instead of using this->_timeOffset
String NTPClient::getFormattedDate(unsigned long secs) {
  unsigned long rawTime = (secs ? secs : this->getEpochTime()) / 86400L;  // in days
  unsigned long days = 0, year = 1970;
  uint8_t month;
  static const uint8_t monthDays[]={31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  while((days += (LEAP_YEAR(year) ? 366 : 365)) <= rawTime)
    year++;
  rawTime -= days - (LEAP_YEAR(year) ? 366 : 365); // now it is days in this year, starting at 0
  days=0;
  for (month=0; month<12; month++) {
    uint8_t monthLength;
    if (month==1) { // february
      monthLength = LEAP_YEAR(year) ? 29 : 28;
    } else {
      monthLength = monthDays[month];
    }
    if (rawTime < monthLength) break;
    rawTime -= monthLength;
  }
  String monthStr = ++month < 10 ? "0" + String(month) : String(month); // jan is month 1
  String dayStr = ++rawTime < 10 ? "0" + String(rawTime) : String(rawTime); // day of month
  return String(year) + "-" + monthStr + "-" + dayStr + "T" + this->getFormattedTime(secs ? secs : 0) + "Z";
}

void NTPClient::end() {
  this->_udp->stop();

  this->_udpSetup = false;
}

void NTPClient::setTimeOffset(int timeOffset) {
  this->_timeOffset     = timeOffset;
}

void NTPClient::setUpdateInterval(unsigned long updateInterval) {
  this->_updateInterval = updateInterval;
}

unsigned long NTPClient::getOffset() {
  return this->_offsetSecs;
}

unsigned long NTPClient::getOffsetMicros() {
  return this->_offsetMicros;
}

bool NTPClient::offsetNegative() {
  return this->_offsetNeg;
}

unsigned long NTPClient::getDelay() {
  return this->_delaySecs;
}

unsigned long NTPClient::getDelayMicros() {
  return this->_delayMicros;
}

void NTPClient::sendNTPPacket() {
  // set all bytes in the buffer to 0
  memset(this->_packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  this->_packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  this->_packetBuffer[1] = 0;     // Stratum, or type of clock
  this->_packetBuffer[2] = 6;     // Polling Interval
  this->_packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  this->_packetBuffer[12]  = 0x49;
  this->_packetBuffer[13]  = 0x4E;
  this->_packetBuffer[14]  = 0x49;
  this->_packetBuffer[15]  = 0x52;

  // set the origin timestamp
  unsigned long ntpSecs;
  unsigned long ntpFrac;
#ifdef DEBUG_NTPClient
  Serial.print("Current epoch = ");
  Serial.print(this->getEpochTime());
  Serial.print(" seconds and ");
  Serial.print(this->getEpochMicros());
  Serial.println(" microseconds");
#endif
  if (this->_currentEpoc != 0) {
    ntpSecs = this->getEpochTime() + SEVENZYYEARS;
    ntpFrac = (unsigned long) (this->getEpochMicros() * 4294.967296);
  } else {
    ntpSecs = 0;
    ntpFrac = (unsigned long) (micros() * 4294.967296);
  }
  //ntpFrac ^= random(1 << 12); // Bottom 12 bits should be random to prevent cumulative roundoff errors
  this->_packetBuffer[24] = (ntpSecs >> 24) & 0xFF;
  this->_packetBuffer[25] = (ntpSecs >> 16) & 0xFF;
  this->_packetBuffer[26] = (ntpSecs >> 8) & 0xFF;
  this->_packetBuffer[27] = ntpSecs & 0xFF;

  this->_packetBuffer[28] = (ntpFrac >> 24) & 0xFF;
  this->_packetBuffer[29] = (ntpFrac >> 16) & 0xFF;
  this->_packetBuffer[30] = (ntpFrac >> 8) & 0xFF;
  this->_packetBuffer[31] = ntpFrac & 0xFF;

  this->extractTimestampAt(24, this->_sentSecs, this->_sentMicros);
#ifdef DEBUG_NTPClient
  Serial.print("Sent t0 as ");
  Serial.println(tmToStr(this->_sentSecs, this->_sentMicros, false));
#endif

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  this->_udp->beginPacket(this->_poolServerName, 123); //NTP requests are to port 123
  this->_udp->write(this->_packetBuffer, NTP_PACKET_SIZE);
  if (this->_udp->endPacket() == 0) {
#ifdef DEBUG_NTPClient
    Serial.println("Failed to send NTP packet");
#endif
  }
}

void NTPClient::setEpochTime(unsigned long secs) {
  this->_currentEpoc = secs;
}
