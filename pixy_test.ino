#include <Pixy2.h>

Pixy2 pixy;

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pixy.init();
  pinMode(LED_BUILTIN, OUTPUT); // Set the built-in LED pin as an output
}

void loop() {
  int i;
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    for (i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 2) { // Use m_signature instead of signature
        blinkLED(); // Call the function to blink the LED
      }
    }
  }
}

void blinkLED() {
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED
  delay(200); // Delay for 1 second
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
