#include <DhtIcap.h>

#define DHT_DATA PB0   // PIN 8 on Uno
#define DHT_POWER A0
#define DHT_TYPE 22

void myErrCallback(const uint8_t status) {
  Console.print("DHT Status: ");
  if ( !status )
    Console.println("OK");
  if ( status & DHT_ICAP_ERR_SUM )
    Console.println("Checksum Error");
  if ( status & DHT_ICAP_ERR_CNT )
    Console.println("Too few bits from sensor (timeout)");
  if ( status & DHT_ICAP_ERR_TONE )
    Console.println("Edges polytonic");
  if ( status & DHT_ICAP_ERR_LINE )
    Console.print("Unresolved bit");
  if ( status & DHT_ICAP_ERR_CAL )
    Console.println("Data timing calibtation failed");
  if ( status & DHT_ICAP_ERR_OVR )
    Console.println("Data overflow (too many bits)");
  if ( status & DHT_ICAP_ERR_INIT )
    Console.println("DhtIcap is uninitialized - run <YourDhtIcapVar>.begin()?");
}

// Instantiate
DhtIcap dht(DHT_TYPE, myErrCallback, 8);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (Serial.available())
    Serial.read();
  while (!Serial.available()) {
    Console.println("Press any key to start.");
    delay (1000);
  }
  Console.println("DhtIcap Example sketch");
  Console.print("Version: ");
  Console.println(DHT_ICAP_VERSION);
  pinMode(DHT_DATA, OUTPUT);
  digitalWrite(DHT_DATA, LOW);
  pinMode(DHT_POWER, OUTPUT);
  digitalWrite(DHT_POWER, LOW);
  digitalWrite(DHT_POWER, HIGH);
  delay(200);
  digitalWrite(DHT_DATA, HIGH);
  if (!dht.begin()) {
    Console.println("DHT not working");
  } else {
    Console.println("Initialized.");
  }
}


void loop() {
  Serial.print("Main Loop");
  delay(2200);

  DhtIcap::Data data;

  if (dht.read(&data, 'F', 'C')) {
    // do the send
    Console.print("T = ");
    Console.print(data.temperature.val);
    Console.println(data.temperature.unit);
    Console.print("H = ");
    Console.print(data.humidity);
    Console.println('%');
    Console.print("D = ");
    Console.print(data.dewpoint.val);
    Console.println(data.dewpoint.unit);
  } else {
    Console.println("Error!");
    myErrCallback(dht.status());
    // there was an error...
  }


// Or, use the discrete functions
//
//  float f, c, k, h;
//  double d, D;
//  f = dht.fahrenheit();
//  h = dht.humidity();
//  c = dht.celsius();
//  k = dht.kelvin();
//  d = dht.dewPoint();
//  D = dht.dewPoint();
  //  Console.print("T = ");
  //  Console.print(f); Console.print("F : ");
  //  Console.print(c); Console.print("C : ");
  //  Console.print(k); Console.println('K');
  //  Console.print("H = ");
  //  Console.print(h); Console.println('%');
  //  Console.print("d = "); Console.print(d); Console.print("C : ");
  //  Console.print(d * 9.0 / 5.0 + 32.0); Console.println('F');
  //  Console.print("D = "); Console.print(d); Console.print("C : ");
  //  Console.print(D * 9.0 / 5.0 + 32.0); Console.println("F\n");

}






