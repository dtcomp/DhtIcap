## Timer1 ICAP Interrupt driven DHT11/21/22 sensor library for Arduino (Uno)

### Read DHT11/21/22 sensors with Timer1 input-capture interrupt.

Mike Figley, dTC - mfigley@gmail.com

| VERSION: | 0.0.1
|
| PURPOSE: | Read temperature/humidity serial data from sensor, fast. |
| LICENSE: | GPL v3 (http://www.gnu.org/licenses/gpl.html) |

### BASED ON:

*   [InputCapture.ino](https://gist.github.com/mpflaga/4404996) interrupt code by [https://github.com/mpflaga](https://github.com/mpflaga)
*   DHT11 interrupt library [https://github.com/niesteszeck](https://github.com/niesteszeck)
*   DHT11 library [http://playground.arduino.cc/Main/DHT11Lib](http://playground.arduino.cc/Main/DHT11Lib)
*   PietteTech_DHT Library [https://github.com/piettetech/PietteTech_DHT](https://github.com/piettetech/PietteTech_DHT/)

While the above DHT library methods may use interrupt-driven techniques, this library takes advantage of input-capture features on your avr-like uC, which greatly simplifies reading from DHT11 sensors. **Works on Arduino Uno Pin 8 only!**

* * *

## Advantages

*   Non-blocking - your sketch can keep running.
*   Speed - Readings occur < 10mS. Although DHT sensors update ~every 2 seconds, accessing data is quick.
*   Efficient - Since all data is captured with ICAP/OVF interrupt, minimal processing is involved in capturing serial data.
*   Convenient - Usage is simple and easy.
*   Accurate - With 1/8 prescale, data edges times have better than 1uS resolution. (F_CPU=16MHz)
*   Reliable - Since data capture is entirely interrupt driven and efficient, potential resource issues are avoided.

## Potential Issues

*   Only works on Pin 8, as this is the only ICAP pin available (Arduino). There are NO options for using a different pin. (Pin can be changed in software with a #define, though)
*   Uses 16-bit Timer1, Other libraries could conflict if they mess with Timer1.
*   Defines two interrupt handlers: TIMER1_OVF_vect (signals "done"), TIMER1_CAPT_vect (captures data)
*   Kinda big - code-wise. Size could be reduced by removing some convenience functionality.
