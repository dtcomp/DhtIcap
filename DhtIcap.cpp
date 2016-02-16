/***
  Based on InputCapture.ino

  Timer 1 high-resolution timing facility.

  Copyright (C) 2008-2012 Bill Roy

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

***/

/***

        We use Interrupt vectors:
        TIMER1_OVF_vect
        TIMER1_CAPT_vect


***/


#include "Arduino.h"
#include <avr/interrupt.h>
#include "math.h"
#include "DhtIcap.h"


uint16_t DhtIcap_event_buffer[DHT_ICAP_MAX_EVENT_BUFF];
uint16_t DhtIcap_overruns=0;
uint8_t  DhtIcap_edges=0;

// Timer Overflow handler

ISR(TIMER1_OVF_vect) {
  TCCR1B = 0; // stop clock
}


// Pin event capture handler

ISR(TIMER1_CAPT_vect) {
  register uint8_t hi,lo;
  
  lo = ICR1L;    // grab captured timer value hi byte
  hi = ICR1H;    // grab captured timer value lo byte
    
  if (! (TIFR1 & 0x1) ) {  // no OVF
    uint16_t w = (hi<<8 | lo);
    DhtIcap_event_buffer[DhtIcap_edges++] = w;
    if (DhtIcap_edges >= DHT_ICAP_MAX_EVENT_BUFF ) {
      TCNT1 = DHT_ICAP_TIMER1_MAX-1; // force OVF
    }
  } 
  else
    ++DhtIcap_overruns;
}



DhtIcap::DhtIcap(uint8_t sensor_type, uint8_t clock_divisor,void (*errCallback)(uint8_t)):
    _errCallback(errCallback),
    _clock(clock_divisor),
    _status(DHT_ICAP_ERR_INIT),
    _sigPin(DHT_ICAP_DATA_PIN),
    _sensor_type(sensor_type),
    _lastGoodReadTime(0),
    _lastStartTime(0),
    _captures(0),
    _calAttempts(0),
    _checksumErrors(0)
{
    pinMode(_sigPin, OUTPUT);
    digitalWrite(_sigPin, HIGH);
    _pulse = (sensor_type == DHT11) ? 20 : 2;
    _preload = DHT_ICAP_TIMER1_MAX - 
 ( (double) (DHT_ICAP_MAX_EVENT_BUFF) * DHT_ICAP_ONE_BIT_TIME) /
 ( (double) clock_divisor/(double)F_CPU);
}

bool DhtIcap::read()
{ 
  unsigned long curtime=millis();

  // didit roll?
  if ( curtime < _lastStartTime )
    _lastGoodReadTime=0;
  _lastStartTime = curtime;

  if ( curtime - _lastGoodReadTime < 2000 )
    return true;

  for (int i = 0; i<DHT_ICAP_MAX_TRIES; i++) {
    _status=0;   // start good
    startCapture();
    if (validEdges()) {
      if (assemble()) {
        _lastGoodReadTime = millis();
        return true;
      }
#ifdef DHT_ICAP_VERB
      else statusPrint(Serial);
#endif
    } 
#ifdef DHT_ICAP_VERB
    else statusPrint(Serial);
#endif
  }

  registerErr(_status);
  return false;
}

void DhtIcap::startCapture(void) {
  uint8_t tccr1b=0b00000010; // div8 
  ++_captures;
  DhtIcap_overruns = 0;
  DhtIcap_edges = 0;
  
  switch (_clock) {
    case (1):
      tccr1b=0b00000001; break;
    case (8):
      tccr1b=0b00000010; break;
    case (64):
      tccr1b=0b00000011; break;
    case (256):
      tccr1b=0b00000100; break;
    case (1024):
      tccr1b=0b00000111; break;
  }
  TCCR1B = 0; // stop clock
  TCCR1A = 0;
  TCCR1C = 0;
  TIMSK1 = 0;
  TIFR1 |= (1<<ICF1)|(1<<OCF1B)|(1<<OCF1A)|(1<<TOV1); // reset flags
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1); // and enable ints
  TCNT1=_preload;
  DDRB |= _BV (0);    // pinMode (8, OUTPUT)
  PORTB &= ~_BV (0);  // digitalWrite (8, LOW)
  delay(_pulse);
  PORTB |= _BV (0);   // digitalWrite (8, HIGH)
  DDRB &= ~_BV (0);   // pinMode (8, INPUT);
  TCCR1B = tccr1b;    // enable clock
  while ( TCCR1B ) delay(1);  // wait for clock to stop
}


void DhtIcap::begin() {
#ifdef DHT_ICAP_VERB
  Serial.println("DhtIcap Init..");
#endif
  delay(2500);
  startCapture();  // Warmup?
  if ( calibrate() ) {
    _status=0;
#ifdef DHT_ICAP_VERB
    Serial.print("Init Succeeds")
#endif
  }
#ifdef DHT_ICAP_VERB 
else
    Serial.println("Init fails!");
#endif
    registerErr(_status);
}

void DhtIcap::registerErr(const uint8_t status) const {
  if (_errCallback ) {
    (_errCallback)(status);
  }
}

bool DhtIcap::validEdges() {

  // enough edges captured? ( Min == BITS + 1 )
  if (DhtIcap_edges <= DHT_ICAP_READ_BITS) {
    _status |= DHT_ICAP_ERR_CNT;
    _edges = DhtIcap_edges;
    return false;
  }

  // Too many edges?
  if (DhtIcap_overruns) {
    _status |= DHT_ICAP_ERR_OVR;
    _overruns = DhtIcap_overruns;
    return false;
  }

  // values monotonic?
  for (int i=0; i<DhtIcap_edges-1; i++) {
    if ( DhtIcap_event_buffer[i] >= DhtIcap_event_buffer[i+1] ) {
       _status |= DHT_ICAP_ERR_TONE;
       return false;
       break;
    }
  }

  return true;
}


bool DhtIcap::calibrate(){
#ifdef DHT_ICAP_VERB
  Serial.println("Find data timing ...");
#endif
  _calAttempts++;
  for (int i=0; i<DHT_ICAP_MAX_TRIES; i++) {
    delay(1000);
    startCapture();
    if ( validEdges() ) {
      calcLine();
      return true;
    }
#ifdef DHT_ICAP_VERB
 else {
      statusPrint(Serial);
    }
#endif
  }
  registerErr(_status |= DHT_ICAP_ERR_CAL);
  return false;
}


void DhtIcap::calcLine(){
  uint16_t dmax = 0, dmin = ~0, davg, dtot=0;
  uint8_t b_last = DhtIcap_edges - 1;
  uint8_t b_first = b_last - DHT_ICAP_READ_BITS;
  volatile uint16_t *p, *q;
  int i;

  for (i = b_last, 
       p = &DhtIcap_event_buffer[b_last], 
       q = &DhtIcap_event_buffer[b_last - 1];
       i > b_first;
       i--, p--, q--)
  {
    uint16_t d = *p - *q;
    dtot += d;
    (d > dmax) ? dmax = d : dmax;
    (d < dmin) ? dmin = d : dmin;
  }

  davg = dtot / (uint16_t) (DHT_ICAP_READ_BITS);
  _line = (dmax + dmin + davg) / 3;
#ifdef DHT_ICAP_DEBUG
  Serial.print("min: "); Serial.println(dmin);
  Serial.print("max: "); Serial.println(dmax);
  Serial.print("avg: "); Serial.println(davg);
  Serial.print("Line: "); Serial.print(_line);
#endif
}

bool DhtIcap::assemble() {
  uint8_t result[5];
  byte b_last = DhtIcap_edges - 1;
  byte b_first = b_last - DHT_ICAP_READ_BITS;
  uint8_t csum=0;  
  uint16_t *t1=&DhtIcap_event_buffer[b_first];
  uint16_t *t2=&DhtIcap_event_buffer[b_first+1];

  for (int bytes = 0; bytes < DHT_ICAP_READ_BYTES; bytes++) {
    uint8_t x=0;
    for (int b = 7; b >=0 ; b--,++t1,++t2) {
      uint16_t d = *t2 - *t1;
      if ( d == _line ) {
        _status |= DHT_ICAP_ERR_LINE;
        _badbit = bytes*8 + b;
        return false;
      }
      if (d > _line) x |= 1 << b; // just set the ones
    }
    result[bytes]=x;
    bytes < DHT_ICAP_CSUM_BYTE ? csum += x : 0;
  }

  if ( result[DHT_ICAP_CSUM_BYTE] != csum ) {
    _status |= DHT_ICAP_ERR_SUM;
    ++_checksumErrors;
    return false;
  }

  uint16_t rht = (result[0]<<8) | result[1];
  uint16_t tht = (result[2]<<8) | result[3];
  _rh = (float) rht / 10.0;
  _tp = (float) tht / 10.0;

  return true;
}


float DhtIcap::celsius() {
    read();
    return _tp;
}

float DhtIcap::humidity() {
    read();
    return _rh;
}

float DhtIcap::fahrenheit() {
    read();
    return _tp * 9.0 / 5.0 + 32.0;
}

float DhtIcap::kelvin() {
    read();
    return _tp + 273.15;
}

/*
 * Added methods for supporting Adafruit Unified Sensor framework
 */
float DhtIcap::readTemperature() {
    return celsius();
}

float DhtIcap::readHumidity() {
    return humidity();
}

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double DhtIcap::dewPoint() {
    read();
    double a = 17.271;
    double b = 237.7;
    double temp  = (a * (double) _tp) / (b + (double) _tp) + log( (double) _rh/100);
    double Td = (b * temp) / (a - temp);
    return Td;
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
double DhtIcap::dewPointSlow() {
    read();
    double a0 = (double) 373.15 / (273.15 + (double) _tp);
    double SUM = (double) -7.90298 * (a0-1.0);
    SUM += 5.02808 * log10(a0);
    SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/a0)))-1) ;
    SUM += 8.1328e-3 * (pow(10,(-3.49149*(a0-1)))-1) ;
    SUM += log10(1013.246);
    double VP = pow(10, SUM-3) * (double) _rh;
    double T = log(VP/0.61078); // temp var
    return (241.88 * T) / (17.558-T);
}

#ifdef DHT_ICAP_VERB
void DhtIcap::statusPrint(HardwareSerial& device){
  device.println("<Status>");
  if ( !_status ) {
    device.println("OK");
  }
  if ( _status & DHT_ICAP_ERR_SUM ) {
    device.print("Checksum Error # "); device.println(_checksumErrors);
  }
  if ( _status & DHT_ICAP_ERR_CNT ) {
    device.print("Too few bits from sensor (timeout) : "); device.println(_edges);
  }
  if ( _status & DHT_ICAP_ERR_TONE ) {
    device.println("Edges polytonic");
  }
  if ( _status & DHT_ICAP_ERR_LINE ) {
    device.print("Unresolved bit: "); device.println(_badbit);
  }
  if ( _status & DHT_ICAP_ERR_CAL ) {
    device.println("Data timing calibtation failed");
  }
  if ( _status & DHT_ICAP_ERR_INIT ) {
    device.println("DhtIcap is uninitialized - run <YourDhtIcapVar>.begin()?");
  }
  if ( _status & DHT_ICAP_ERR_OVR ) {
    device.println("Data overflow (too many bits)");
  }
  device.println("</Status>");
}
#endif


