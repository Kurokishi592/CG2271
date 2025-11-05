const int NEW_TX_PIN = 1;
const int NEW_RX_PIN = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN);
  delay(1000);
  Serial.print("UART DEMO\n\n");
}

void loop() {
  Serial1.println("Hello this is ESP32");

  // Wait for data
  while(Serial1.available() <=0);

  String x = Serial1.readString();
  Serial.println(x);
  delay(1000);
}

