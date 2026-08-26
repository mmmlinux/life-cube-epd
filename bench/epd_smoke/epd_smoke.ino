// SPDX-License-Identifier: GPL-3.0-or-later
// Smoke test: does LilyGo-EPD47 build and drive the panel on core 3.3.11?
#include <Arduino.h>
#include "epd_driver.h"
#include "ed047tc1.h"

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("smoke: epd_base_init");
  epd_base_init(EPD_WIDTH);
  epd_poweron();
  uint32_t t0 = millis();
  epd_clear();
  Serial.printf("smoke: epd_clear took %lu ms\n", (unsigned long)(millis() - t0));
  Rect_t a = { .x = 200, .y = 150, .width = 560, .height = 240 };
  t0 = millis();
  epd_push_pixels(a, 200, 0);   // darken the box
  Serial.printf("smoke: push_pixels took %lu ms\n", (unsigned long)(millis() - t0));
  epd_poweroff();
  Serial.println("smoke: done");
}

void loop() { delay(1000); }
