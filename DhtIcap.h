/*
 * FILE:        DhtIcap.h
 * VERSION:     0.1
 * PURPOSE:     ICAP driven lib for DHT sensors
 * LICENSE:     GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *
 * Mike Figley  mfigley@gmail.com
 *
 * Based on InputCapture.ino code by https://gist.github.com/mpflaga/4404996
 * Based on adaptation by S Piette (Piette Technologies) scott.piette@gmail.com
 * Based on adaptation by niesteszeck (github/niesteszeck)
 * Based on original DHT11 library (http://playgroudn.adruino.cc/Main/DHT11Lib)
 *
 *
 * With this library connect the DHT sensor to PIN 8 (only)
 *
 */

#ifndef __DHT_ICAP_H__
#define __DHT_ICAP_H__

#include "application.h"

#define DHT_ICAP_VERSION "0.1"

#ifndef DHT_ICAP_DATA_PIN
#define DHT_ICAP_DATA_PIN PB0
#endif

// device types
#define DHT11                               11
#define DHT21                               21
#define AM2301                              21
#define DHT22                               22
#define AM2302                              22

#define DHT_ICAP_STATUS_GOOD                0
#define DHT_ICAP_MAX_TRIES                  10

// error codes
#define DHT_ICAP_ERR_SUM                   1<<0
#define DHT_ICAP_ERR_CNT                   1<<1
#define DHT_ICAP_ERR_TONE                  1<<2
#define DHT_ICAP_ERR_LINE                  1<<3
#define DHT_ICAP_ERR_CAL                   1<<4
#define DHT_ICAP_ERR_INIT                  1<<5
#define DHT_ICAP_ERR_OVR                   1<<6


// DHT22/Am2302 : 5 bytes / 40 bits
#define DHT_ICAP_READ_BYTES 5
#define DHT_ICAP_READ_BITS DHT_ICAP_READ_BYTES*8
#define DHT_ICAP_MAX_EVENT_BUFF DHT_ICAP_READ_BITS+8  // and some room
#define DHT_ICAP_CSUM_BYTE DHT_ICAP_READ_BYTES-1
#define DHT_ICAP_ONE_BIT_TIME 0.00013                 // 130uS Max from Am2302
#define DHT_ICAP_TIMER1_MAX 0xffff

extern uint16_t DhtIcap_event_buffer[DHT_ICAP_MAX_EVENT_BUFF];
extern uint16_t DhtIcap_overruns;
extern uint8_t  DhtIcap_edges;

class DhtIcap
{
public:
    const double oneBitTime = DHT_ICAP_ONE_BIT_TIME;   // 130uS for a 'one'
    const double maxBitTime = (double)(DHT_ICAP_MAX_EVENT_BUFF)*oneBitTime; // Time for 40+ bits
    DhtIcap(uint8_t sensor_type, uint8_t clock_divisor=8, void (*errCallback)(uint8_t)=0);
    uint8_t status(){ return _status; }
    bool valid(){ return _status == 0; }
    void begin();
    bool calibrate();
    bool read();
    void startCapture();
    float celsius();
    float fahrenheit();
    float kelvin();
    double dewPoint();
    double dewPointSlow();
    float temperature();
    float humidity();
    float readTemperature();
    float readHumidity();
    void setClock(uint16_t div){ _clock = div; }


#ifdef DHT_ICAP_VERB
    void statusPrint(HardwareSerial&);
#endif

private:
    void (*_errCallback)(const uint8_t);
    uint16_t _clock;
    volatile uint8_t _status;
    uint8_t _sigPin;
    uint8_t _sensor_type;
    unsigned long _lastGoodReadTime;
    unsigned long _lastStartTime;    
    unsigned long _captures;
    uint16_t _calAttempts;
    uint32_t _checksumErrors;
    uint16_t _pulse;
    uint16_t _edges;
    uint16_t _line;
    uint8_t  _badbit;
    uint16_t _preload;
    uint16_t _overruns;
    float _rh;
    float _tp;    
    bool assemble();
    void calcLine();
    bool validEdges();
    void registerErr(const uint8_t) const;
};
#endif
