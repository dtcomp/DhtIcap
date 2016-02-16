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

        We define Interrupt handlers for:
        TIMER1_OVF_vect
        TIMER1_CAPT_vect


***/


#include "Arduino.h"
#include <avr/interrupt.h>
#include "math.h"
#include <SoftwareSerial.h>
#include "DhtIcap.h"

extern SoftwareSerial& Console;

uint16_t DhtIcap::event_buffer[DHT_ICAP_MAX_EVENT_BUFF];
volatile uint16_t DhtIcap::overruns=0;
volatile uint8_t  DhtIcap::edges=0;
bool DhtIcap::fastDewpoint=true;

unsigned char reg[10];

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
    DhtIcap::event_buffer[DhtIcap::edges++] = w;
    if (DhtIcap::edges >= DHT_ICAP_MAX_EVENT_BUFF ) {
      TCNT1 = DHT_ICAP_TIMER1_MAX-1; // force OVF
    }
  } 
  else
    ++DhtIcap::overruns;
}


DhtIcap::DhtIcap(uint8_t sensor_type, void (*errCallback)(uint8_t), uint8_t clock_divisor):
    _errCallback(errCallback),
    _clock(clock_divisor),
    _status(DHT_ICAP_ERR_INIT),
    _sigPin(DHT_ICAP_DATA_PIN),
    _sensor_type(sensor_type),
    _lastGoodReadTime(0),
    _lastStartTime(0),
    _captures(0),
    _calAttempts(0),
    _checksumErrors(0),
    _seq(0)
{
    pinMode(_sigPin, OUTPUT);
    digitalWrite(_sigPin, HIGH);
    _pulse = (sensor_type == DHT11) ? 20 : 2;
    _preload = DHT_ICAP_TIMER1_MAX - 
 ( (double)(DHT_ICAP_MAX_EVENT_BUFF) * DHT_ICAP_1BIT_TIME_MAX) /
 ( (double)clock_divisor/(double)F_CPU);
 _lineInit = (double)DHT_ICAP_STROBE/( (double)clock_divisor/(double)F_CPU);
}

bool DhtIcap::read(bool force)
{
  unsigned long curtime=millis();

  // didit roll?
  if ( curtime < _lastStartTime )
    _lastGoodReadTime=0;
  _lastStartTime = curtime;

  if ( (curtime - _lastGoodReadTime < 2000) && !force)
    return true;

  for (int i = 0; i<DHT_ICAP_MAX_TRIES; i++) {
    _status=0;   // start good
    startCapture();
    if (validEdges()) {
      if (assemble()) {
        _lastGoodReadTime = millis();
        return true;
      }
    } 
  }
  registerErr(_status);
  return false;
}

#ifdef DHT_ICAP_DEBUG
void DhtIcap::dumpEdges(){
  Serial.print("Dump edges"); Serial.println(_captures);
  byte b_last = DhtIcap::edges - 1;
  byte b_first = b_last - DHT_ICAP_READ_BITS;
  uint16_t *t1=&DhtIcap::event_buffer[b_first];
  uint16_t *t2=&DhtIcap::event_buffer[b_first+1];
  
  for (int i=b_first,j=1; i<b_last; i++,j++,t1++,t2++) {
	  int d = *t2 - *t1;
	  Serial.print(j); Serial.print(": "); Serial.print(*t1);
	  Serial.print(" - "); Serial.print(*t2); Serial.print(" -> ");
	  Serial.println(d);
  }
}
#endif

void DhtIcap::startCapture(void) {
  uint8_t tccr1b=0b00000010; // div8 
  ++_captures;
  DhtIcap::overruns = 0;
  DhtIcap::edges = 0;
  
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
#ifdef DHT_ICAP_DEBUG
  dumpEdges();
#endif
}


bool DhtIcap::begin() {
  delay(2200);
  startCapture();  // Warmup?
  if ( calibrate() ) {
    _status=0;
    return true;
  }
  registerErr(_status);
  return false;
}

void DhtIcap::registerErr(const uint8_t status) const {
  if (_errCallback ) {
    (_errCallback)(status);
  }
}

bool DhtIcap::validEdges() {

  // enough edges captured? ( Min == BITS + 1 )
  if (edges <= DHT_ICAP_READ_BITS) {
    _status |= DHT_ICAP_ERR_CNT;
    _edges = edges;
    return false;
  }

  // Too many edges?
  if (DhtIcap::overruns) {
    _status |= DHT_ICAP_ERR_OVR;
    _overruns = DhtIcap::overruns;
    return false;
  }

  // values monotonic?
  for (int i=0; i<DhtIcap::edges-1; i++) {
    if ( DhtIcap::event_buffer[i] >= DhtIcap::event_buffer[i+1] ) {
       _status |= DHT_ICAP_ERR_TONE;
       return false;
    }
  }
  return true;
}


bool DhtIcap::calcLine(){
  uint8_t o=0, z=0;
  uint16_t dmax = 0, dmin = ~0,
        davg, davg1, davg0,
        dtot=0, dtot1=0, dtot0=0;
  uint8_t b_last = DhtIcap::edges - 1;
  uint8_t b_first = b_last - DHT_ICAP_READ_BITS;
  uint16_t *p, *q;
  int i;

  for (i = b_last, 
       p = &DhtIcap::event_buffer[b_last], 
       q = &DhtIcap::event_buffer[b_last - 1];
       i > b_first;
       i--, p--, q--)
  {
    uint16_t d = *p - *q;
    dtot += d;
    (d > dmax) ? dmax = d : 0;
    (d < dmin) ? dmin = d : 0;
    (d > _lineInit) ? dtot1+=d,o++ : 0;
    (d < _lineInit) ? dtot0+=d,z++ : 0;
  }

  davg = dtot / (uint8_t) (DHT_ICAP_READ_BITS);
  davg1 = dtot1/o;
  davg0 = dtot0/z;
  _line = (davg1 + davg0)/2;
  
#ifdef DHT_ICAP_DEBUG
  Serial.print("Strobe uS: "); Serial.print(DHT_ICAP_STROBE*1000000.0);
  Serial.print("min: "); Serial.println(dmin);
  Serial.print("max: "); Serial.println(dmax);
  Serial.print("avg: "); Serial.println(davg);
  Serial.print("avg1: "); Serial.println(davg1);
  Serial.print("avg0: "); Serial.println(davg0);
  Serial.print("1s: "); Serial.println(o);
  Serial.print("0s: "); Serial.println(z);
  Serial.print("Old Line: "); Serial.println(_lineInit);
#endif

  if ((dmax > _line) && (_line > dmin)) {
    _lineInit = _line;
#ifdef DHT_ICAP_DEBUG
      Serial.print("New Line: "); Serial.println(_lineInit);
#endif
    return true;
  }
  return false;
}


bool DhtIcap::calibrate(){

  for (int i=0; i<DHT_ICAP_MAX_TRIES; i++) {
    _calAttempts++;
    int pass=0;
    for (int j=0; j<=3; j++) {
      delay(500);
      startCapture();
      if (validEdges())
        if (calcLine())
          ++pass;
    }
    if (pass>=3)
      return true;
  }
  registerErr(_status |= DHT_ICAP_ERR_CAL);
  return false;
}


bool DhtIcap::assemble() {
  uint8_t result[5];
  byte b_last = DhtIcap::edges - 1;
  byte b_first = b_last - DHT_ICAP_READ_BITS;
  uint8_t csum=0;  
  uint16_t *t1=&DhtIcap::event_buffer[b_first];
  uint16_t *t2=&DhtIcap::event_buffer[b_first+1];

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
  _tp_C = (float) tht / 10.0;

  return true;
}


float DhtIcap::convert(float t, char from, char to){
  if (to == from) return t;
  
  switch(from) {
		case 'F': {
    float tmp = t * 5.0 / 9.0 - 32.0;
		switch(to) {
			case 'C':
			  return tmp;
			case 'K':
			  return tmp + 273.1;
		}
  }
		case 'C': {
			switch(to) {
			case 'F':
			return t * 9.0 / 5.0 + 32.0;
			case 'K':
			  return t + 273.15;
		}
  }
		case 'K': {
		  float tmp = t - 273.1;
			switch(to) {
			case 'F':
			  return tmp * 9.0 / 5.0 + 32.0;
			case 'C':
			  return tmp;
			}
    }
	}
	return -99999.9;
}

bool DhtIcap::read(Data *d, char tunit, char dunit){
	Serial.println("in readdata");
	if (read(true)){
	  d->seq = _seq++;  // update this here only!
	  d->read = _lastGoodReadTime;
	  d->temperature.val = convert(_tp_C,'C',tunit);
	  d->temperature.unit = tunit;
	  d->dewpoint.val = dewPointCalc(_tp_C,_rh,dunit);
	  d->dewpoint.unit = dunit;
	  d->humidity = _rh;
	  return true;
  }
  d->read = 0;   // signals error condition
  d->seq = 0;
	return false;
}

float DhtIcap::celsius() {
	read();
  return _tp_C;
}

float DhtIcap::humidity() {
	read();
  return _rh;
}

float DhtIcap::fahrenheit() {
  read();
  return convert(_tp_C,'C','F');
}

float DhtIcap::kelvin() {
  read();
  return convert(_tp_C,'C','F');
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
float DhtIcap::dewPointFast(float tp_c, float rh, char unit) {
    double a = 17.271;
    double b = 237.7;
    double temp  = (a * (double) tp_c) / (b + (double) tp_c) + log( (double) rh/100);
    return convert((b * temp) / (a - temp), 'C', unit);
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
float DhtIcap::dewPointSlow(float tp_c, float rh, char unit) {
    double a0 = (double) 373.15 / (273.15 + (double) tp_c);
    double SUM = (double) -7.90298 * (a0-1.0);
    SUM += 5.02808 * log10(a0);
    SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/a0)))-1) ;
    SUM += 8.1328e-3 * (pow(10,(-3.49149*(a0-1)))-1) ;
    SUM += log10(1013.246);
    double VP = pow(10, SUM-3) * (double) rh;
    double T = log(VP/0.61078); // temp var
    return convert((241.88 * T) / (17.558-T), 'C', unit);
}

float DhtIcap::dewPointCalc(float tp_C, float rh, char unit){
  return fastDewpoint ? dewPointFast(tp_C,rh,unit) : dewPointSlow(tp_C,rh,unit);
}

float DhtIcap::dewPoint(char unit){
	if (read()) {
	  return fastDewpoint ? dewPointFast(_tp_C,_rh,unit) : dewPointSlow(_tp_C,_rh,unit);
  }
  return -99999.9;
}

#ifdef DHT_ICAP_VERB
void DhtIcap::statusPrint(Stream& device){
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


