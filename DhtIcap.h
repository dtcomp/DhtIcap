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
#define DHT_ICAP_MAX_TRIES                  5

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
#define DHT_ICAP_READ_BITS (DHT_ICAP_READ_BYTES*8)
#define DHT_ICAP_MAX_EVENT_BUFF (DHT_ICAP_READ_BITS+8)  // and some room
#define DHT_ICAP_CSUM_BYTE (DHT_ICAP_READ_BYTES-1)
#define DHT_ICAP_BIT_LOW_TIME_TYP    0.000050
#define DHT_ICAP_0BIT_HIGH_TIME_TYP  0.000026
#define DHT_ICAP_1BIT_HIGH_TIME_TYP  0.000070
#define DHT_ICAP_1BIT_TIME_TYP       0.000120
#define DHT_ICAP_1BIT_TIME_MAX       0.000130
#define DHT_ICAP_0BIT_TIME_TYP       0.000076
#define DHT_ICAP_STROBE (double(DHT_ICAP_1BIT_TIME_TYP+DHT_ICAP_0BIT_TIME_TYP)/2.0)

#define DHT_ICAP_TIMER1_MAX 0xffff

#ifndef DHT_ICAP_UNIT_TP_DEFAULT
#define DHT_ICAP_UNIT_TP_DEFAULT 'C'
#endif

#ifndef DHT_ICAP_UNIT_DP_DEFAULT
#define DHT_ICAP_UNIT_DP_DEFAULT 'C'
#endif


class DhtIcap
{
public:

typedef struct DhtIcapTemp {
    float val;
    char  unit;
} Temp;

typedef struct DhtIcapData {
    Temp  temperature;   // temp struct
    float humidity;      // Relative humidity value
    Temp  dewpoint;      // temp struct
    unsigned long read;  // system time (millis())
    unsigned long seq;   // sequence number of reading
} Data;

    static uint16_t event_buffer[DHT_ICAP_MAX_EVENT_BUFF];
    static volatile uint16_t overruns;
    static volatile uint8_t  edges;


    DhtIcap(uint8_t sensor_type, void (*errCallback)(uint8_t)=0, uint8_t clock_divisor=8);
    
    static float convert(float val, char from, char to);
    static float dewPointFast(float t, float rh, char unit=DHT_ICAP_UNIT_TP_DEFAULT);
    static float dewPointSlow(float t, float rh, char unit=DHT_ICAP_UNIT_TP_DEFAULT);
    static float dewPointCalc(float temp, float humidity, char unit);
    
    uint8_t status(){ return _status; }
    
    bool setTPUnitDefault(char);
    bool setDPUnitDefault(char);
    bool valid(){ return _status == 0; }
    bool calibrate();
    bool read(bool force=false);
    bool read(Data*, char t_unit=DHT_ICAP_UNIT_TP_DEFAULT,
                     char d_unit=DHT_ICAP_UNIT_TP_DEFAULT);
    bool usingFastDewpoint() { return fastDewpoint; }
    float dewPoint(char unit=DHT_ICAP_UNIT_TP_DEFAULT);

    
    float celsius();
    float fahrenheit();
    float kelvin();
    float humidity();
    float readTemperature();
    float readHumidity();
    
    bool begin();
    void setClock(uint16_t div){ _clock = div; }
    void useFastDewpoint(){ fastDewpoint=true; }
    void startCapture();
    void dumpEdges();

#ifdef DHT_ICAP_VERB
    void statusPrint(HardwareSerial&);
#endif

private:
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
    uint16_t _line;
    uint8_t  _badbit;
    uint16_t _preload;
    uint16_t _overruns;
    uint16_t _edges;
    uint16_t _lineInit;
    static bool fastDewpoint;
    unsigned long _seq;
    float _rh;
    float _tp_C;
    bool assemble();
    bool calcLine();
    bool validEdges();
    void registerErr(const uint8_t) const;
    void (*_errCallback)(const uint8_t);
};
#endif
