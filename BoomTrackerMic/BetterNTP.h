#pragma once

#include "Arduino.h"

#include <Udp.h>

#define SEVENZYYEARS 2208988800UL
#define NTP_PACKET_SIZE 48
#define NTP_DEFAULT_LOCAL_PORT 1337
#define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ) )

void addTimes(unsigned long aSecs, unsigned long aMicros, bool aNeg, unsigned long bSecs, unsigned long bMicros, bool bNeg, unsigned long *sumSecs, unsigned long *sumMicros, bool *sumNeg);
void subtractTimes(unsigned long aSecs, unsigned long aMicros, bool aNeg, unsigned long bSecs, unsigned long bMicros, bool bNeg, unsigned long *diffSecs, unsigned long *diffMicros, bool *diffNeg);
void divBy2(unsigned long *secs, unsigned long *microseconds);
const char *tmToStr(unsigned long secs, unsigned long microseconds, bool negative);

class NTPClient {
  private:
    UDP*          _udp;
    bool          _udpSetup       = false;

    const char*   _poolServerName = "pool.ntp.org"; // Default time server
    int           _port           = NTP_DEFAULT_LOCAL_PORT;
    int           _timeOffset     = 0;

    unsigned long _updateInterval = 60000;  // In ms

    unsigned long _currentEpoc    = 0;      // In s
    unsigned long _currentMicros  = 0;      // in microseconds
    unsigned long _microsAtLastEpochFetch = 0; // in microseconds, to save the value that we can't return
    unsigned long _lastUpdate     = 0;      // In microseconds
    unsigned long _delaySecs      = 0;      // round trip delay of last update, in whole seconds
    unsigned long _delayMicros    = 0;      // microseconds part of round trip delay 
    unsigned long _offsetSecs     = 0;      // last measured offset, seconds part
    unsigned long _offsetMicros   = 0;      // last measured offset, microseconds part
    bool          _offsetNeg      = false;
    unsigned long _sentSecs       = 0;
    unsigned long _sentMicros     = 0;

    byte          _packetBuffer[NTP_PACKET_SIZE];

    void          sendNTPPacket();
    bool          isValid(byte * ntpPacket);
    void          extractTimestampAt(int at, unsigned long &secs, unsigned long &microseconds);
 
  public:
    NTPClient(UDP& udp);
    NTPClient(UDP& udp, int timeOffset);
    NTPClient(UDP& udp, const char* poolServerName);
    NTPClient(UDP& udp, const char* poolServerName, int timeOffset);
    NTPClient(UDP& udp, const char* poolServerName, int timeOffset, unsigned long updateInterval);

    /**
     * Starts the underlying UDP client with the default local port
     */
    void begin();

    /**
     * Starts the underlying UDP client with the specified local port
     */
    void begin(int port);

    /**
     * This should be called in the main loop of your application. By default an update from the NTP Server is only
     * made every 60 seconds. This can be configured in the NTPClient constructor.
     *
     * @return true on success, false on failure
     */
    bool update();

    /**
     * This will force the update from the NTP Server.
     *
     * @return true on success, false on failure
     */
    bool forceUpdate();

    int getDay();
    int getHours();
    int getMinutes();
    int getSeconds();

    /**
     * Changes the time offset. Useful for changing timezones dynamically
     */
    void setTimeOffset(int timeOffset);

    /**
     * Set the update interval to another frequency. E.g. useful when the
     * timeOffset should not be set in the constructor
     */
    void setUpdateInterval(unsigned long updateInterval);

    /**
    * @return secs argument (or 0 for current time) formatted like `hh:mm:ss`
    */
    String getFormattedTime(unsigned long secs = 0);

    /**
     * @return time in seconds since Jan. 1, 1970
     */
    unsigned long getEpochTime();

    /**
     * @ return fractional time in microseconds since last whole second at the
     * time of the last call to getEpochTime()
     */
    unsigned long getEpochMicros();

    /**
     * @ return whole second part of last measured offset
     */
    unsigned long getOffset();

    /**
     * @ return microseconds part of last measured offset
     */
    unsigned long getOffsetMicros();

    /**
     * @ return true if offset is negative
     */
    bool offsetNegative();

    /**
     * @ return whole seconds part of round trip delay
     */
    unsigned long getDelay();

    /**
     * @ return microseconds part of round trip delay
     */
    unsigned long getDelayMicros();
     
    /**
    * @return secs argument (or 0 for current date) formatted to ISO 8601
    * like `2004-02-12T15:19:21+00:00`
    */
    String getFormattedDate(unsigned long secs = 0);

    /**
     * Stops the underlying UDP client
     */
    void end();

    /**
    * Replace the NTP-fetched time with seconds since Jan. 1, 1970
    */
    void setEpochTime(unsigned long secs);
};
