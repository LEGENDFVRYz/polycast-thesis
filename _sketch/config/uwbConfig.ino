// ESP32 WROOM version of the UWB configuration code
// Uses UART on GPIO16 (RX) and GPIO17 (TX)

#define UWB_RX 16   // ESP32 RX pin connected to UWB TX
#define UWB_TX 17   // ESP32 TX pin connected to UWB RX

void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting board configuration (ESP32)...");

  // Hardware Serial1 mapped to GPIO16 (RX) and GPIO17 (TX)
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);
  delay(1000);

  // Send configuration
  Serial.println("Sending AT+SETCFG...");
  Serial1.print("AT+SETCFG=1,1,1,1\r\n");
  delay(500);

  if (Serial1.available()) {
    Serial.println("Response:");
    Serial.println(Serial1.readString());
  }

  delay(1500);

  // Save configuration
  Serial.println("Sending AT+SAVE...");
  Serial1.print("AT+SAVE\r\n");
  delay(500);

  if (Serial1.available()) {
    Serial.println("Response:");
    Serial.println(Serial1.readString());
  }

  delay(1000);

  // Verify configuration
  Serial.println("Sending AT+GETCFG...");
  Serial1.print("AT+GETCFG\r\n");
  delay(300);

  if (Serial1.available()) {
    Serial.println("Response:");
    Serial.println(Serial1.readString());
  }

  Serial.println("Configuration process completed.");
}

void loop() {
  // nothing needed
}