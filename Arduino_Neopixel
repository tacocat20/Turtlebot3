#include <Adafruit_NeoPixel.h>

#define PIN 6
#define NUM_LEDS 8

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

char cmd = 'S';

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.clear();
  strip.show();
}

void loop() {
  if (Serial.available()) {
    cmd = Serial.read();
  }

  if (cmd == 'M') {
    policeLights();
  } else {
    strip.clear();
    strip.show();
  }
}

void policeLights() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
  }
  strip.show();
  delay(100);

  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255));
  }
  strip.show();
  delay(100);
}
