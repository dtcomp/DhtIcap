//#define DHT_ICAP_DEBUG
#include <SoftwareSerial.h>
#include <DhtIcap.h>

#define DHT_DATA PB0
#define DHT_POWER A0
#define DHT_TYPE 22

void myErrCallback(const uint8_t status) {
  Serial.print("DHT Error: ");
  if ( !status )
    Serial.println("OK");
  if ( status & DHT_ICAP_ERR_SUM )
    Serial.println("Checksum Error");
  if ( status & DHT_ICAP_ERR_CNT )
    Serial.println("Too few bits from sensor (timeout)");
  if ( status & DHT_ICAP_ERR_TONE )
    Serial.println("Edges polytonic");
  if ( status & DHT_ICAP_ERR_LINE )
    Serial.print("Unresolved bit");
  if ( status & DHT_ICAP_ERR_CAL )
    Serial.println("Data timing calibtation failed");
  if ( status & DHT_ICAP_ERR_OVR )
    Serial.println("Data overflow (too many bits)");
  if ( status & DHT_ICAP_ERR_INIT )
    Serial.println("DhtIcap is uninitialized - run <YourDhtIcapVar>.begin()?");
}

// Instantiate
DhtIcap dht(DHT_TYPE, 8, myErrCallback);
SoftwareSerial Console(10,11);
void setup() {
  Console.begin(9600);
  Console.println("Gotta Console!!!");
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial.available()) {
    Serial.println("Press any key to start.");
    delay (1000);
  }
  Serial.println("DhtIcap Example sketch");
  Serial.print("Version: ");
  Serial.println(DHT_ICAP_VERSION);
  pinMode(DHT_DATA, OUTPUT);
  digitalWrite(DHT_DATA, LOW);
  pinMode(DHT_POWER, OUTPUT);
  digitalWrite(DHT_POWER, LOW);
  digitalWrite(DHT_POWER, HIGH);
  delay(200);
  digitalWrite(DHT_DATA, HIGH);
  dht.begin();
  if (!dht.valid()) {
    Serial.println("DHT not working, try #define DHT_ICAP_VERB");
    Serial.println("and/or #define DHT_ICAP_DEBUG");
    Serial.println("then, dht.statusPrint();");
  }
}

float f, c, k, h;
double d, D;

void loop() {

  f = dht.fahrenheit();
  h = dht.humidity();
  c = dht.celsius();
  k = dht.kelvin();
  d = dht.dewPoint();
  D = dht.dewPointSlow();

  Serial.print("T = ");
  Serial.print(f); Serial.print("F : ");
  Serial.print(c); Serial.print("C : ");
  Serial.print(k); Serial.println('K');
  Serial.print("H = ");
  Serial.print(h); Serial.println('%');
  Serial.print("d = "); Serial.print(d); Serial.print("C : ");
  Serial.print(d * 9.0 / 5.0 + 32.0); Serial.println('F');
  Serial.print("D = "); Serial.print(d); Serial.print("C : ");
  Serial.print(D * 9.0 / 5.0 + 32.0); Serial.println("F\n");
  delay(2000);
}






