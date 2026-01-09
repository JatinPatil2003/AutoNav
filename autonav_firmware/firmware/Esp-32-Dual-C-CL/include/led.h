#include <Arduino.h>

#include <FastLED.h>
#include <AsyncTimer.h>   // async task scheduler

#ifndef LED_PIN
#define LED_PIN 13
#endif

#ifndef NUM_LEDS
#define NUM_LEDS 24
#endif

#ifndef BRIGHTNESS
#define BRIGHTNESS 128
#endif

CRGB leds[NUM_LEDS];
AsyncTimer timer_led;

int led_status = 0;

uint8_t brightnessLevel = 128;
int fadeDirectionFast = 8; // 1 = fade in, -1 = fade out
int fadeDirectionSlow = 3;
bool fading = true;   // true = fade active, false = waiting pause

#define GREEN CHSV(86, 255, 255) // Greenish tone

void Green_OutWait() {
  if (!fading) return;  // Skip if in pause state

  brightnessLevel -= 5;

  // Apply color & brightness
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(86, 255, brightnessLevel); // Greenish tone
  }
  FastLED.show();

  // When fully faded
  if (brightnessLevel <= 5) {
    fading = false; // stop fading

    // Schedule restart after 1000ms WITHOUT blocking
    timer_led.setTimeout([]() {
      brightnessLevel = 255;
      fading = true;
    }, 250);
  }
}

void Red_InOut() {
  // Update brightness
  brightnessLevel -= fadeDirectionFast;

  // Clamp
  if (brightnessLevel >= 245) {
    // brightnessLevel = 255;
    fadeDirectionFast *= -1;   // start fading OUT
  }

  if (brightnessLevel <= 10) {
    fadeDirectionFast *= -1;    // start fading IN
  }

  // Apply color
  fill_solid(leds, NUM_LEDS, CHSV(0, 255, brightnessLevel));
  FastLED.show();
}

void Green_InOut() {
  // Update brightness
  brightnessLevel -= fadeDirectionFast;

  // Clamp
  if (brightnessLevel >= 245) {
    // brightnessLevel = 255;
    fadeDirectionFast *= -1;   // start fading OUT
  }

  if (brightnessLevel <= 10) {
    fadeDirectionFast *= -1;    // start fading IN
  }

  // Apply color
  fill_solid(leds, NUM_LEDS, CHSV(86, 255, brightnessLevel));
  FastLED.show();
}



void Blue_InOut() {
  brightnessLevel -= fadeDirectionSlow;

  // Apply color & brightness
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(172, 255, brightnessLevel); // Greenish tone
  }
  FastLED.show();
  
  if (brightnessLevel >= 245) fadeDirectionSlow *= -1;
  else if (brightnessLevel <= 10) fadeDirectionSlow *= -1;
}

void Violet_InOut() {
  // Update brightness
  brightnessLevel -= fadeDirectionSlow;

  // Clamp
  if (brightnessLevel >= 245) {
    // brightnessLevel = 255;
    fadeDirectionSlow *= -1;   // start fading OUT
  }

  if (brightnessLevel <= 10) {
    fadeDirectionSlow *= -1;    // start fading IN
  }

  // Apply color
  fill_solid(leds, NUM_LEDS, CHSV(194, 255, brightnessLevel));
  FastLED.show();
}

void White_InOut() {
  // Update brightness
  brightnessLevel -= fadeDirectionSlow;

  // Clamp
  if (brightnessLevel >= 245) {
    // brightnessLevel = 255;
    fadeDirectionSlow *= -1;   // start fading OUT
  }

  if (brightnessLevel <= 10) {
    fadeDirectionSlow *= -1;    // start fading IN
  }

  // Apply color
  fill_solid(leds, NUM_LEDS, CHSV(0, 0, brightnessLevel));
  FastLED.show();
}

void Yellow_InOut() {
  // Update brightness
  brightnessLevel -= fadeDirectionSlow;

  // Clamp
  if (brightnessLevel >= 245) {
    // brightnessLevel = 255;
    fadeDirectionSlow *= -1;   // start fading OUT
  }

  if (brightnessLevel <= 10) {
    fadeDirectionSlow *= -1;    // start fading IN
  }

  // Apply color
  fill_solid(leds, NUM_LEDS, CHSV(39, 255, brightnessLevel));
  FastLED.show();
}

int waveindex = 0;
void Blue_Rotate() {
  leds[waveindex] = CHSV(172, 255, 255); // Greenish tone
  fadeToBlackBy(leds, NUM_LEDS, 20);
  FastLED.show();
  EVERY_N_MILLISECONDS(50){
    waveindex += 1;
  }
  if (waveindex >= NUM_LEDS) waveindex = 0;
}

void Cyan_Rotate() {
  leds[waveindex] = CHSV(128, 255, 255); // Greenish tone
  fadeToBlackBy(leds, NUM_LEDS, 12);
  FastLED.show();
  EVERY_N_MILLISECONDS(50){
    waveindex += 1;
  }
  if (waveindex >= NUM_LEDS) waveindex = 0;
}

bool blink_state = false;

void Green_Blink(){
  EVERY_N_MILLISECONDS(400){
    if (blink_state){
      fill_solid(leds, NUM_LEDS, CHSV(86, 255, 255));
    }
    else {
      fill_solid(leds, NUM_LEDS, CHSV(0, 255, 0));
    }
    blink_state = !blink_state;
    FastLED.show();
  }
}

void Off() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void led_timer_callback(){
  switch (led_status) {
    case 0:
      Off();
      break;
    case 1:
      Green_OutWait();
      break;
    case 2:
      Red_InOut();
      break;
    case 3:
      Blue_Rotate();
      break;
    case 4:
      Blue_InOut();
      break;
    case 5:
      Violet_InOut();
      break;
    case 6:
      Cyan_Rotate();
      break; 
    case 7:
      Green_Blink(); 
      break;
    case 8:
      White_InOut();
      break;
    case 9:
      Yellow_InOut();
      break;
    case 10:
      Green_InOut();
      break;

    default:
      break;
  }
}

void led_setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  timer_led.setInterval(led_timer_callback, 20);
}

void led_handle() {
  timer_led.handle();
}
