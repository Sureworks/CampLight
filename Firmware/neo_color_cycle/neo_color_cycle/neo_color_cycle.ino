#include <FastLED.h>
#define DATA_PIN 5
#define NUM_LEDS 7

#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

void setup(){
  FastLED.addLeds<WS2811, DATA_PIN, GRB>(leds, NUM_LEDS);

}
void loop(){

  // Let's take 256 steps to get from blue to red
  // (the most possible with an LED with 8 bit RGB values)

  for( int colorStep=0; colorStep<256; colorStep++ ) {

      int r = colorStep;  // Redness starts at zero and goes up to full
      int b = 255-colorStep;  // Blue starts at full and goes down to zero
      int g = 0;              // No green needed to go from blue to red

      // Now loop though each of the LEDs and set each one to the current color

      for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(r,g,b);
      }



      // Display the colors we just set on the actual LEDs
      FastLED.show();

      delay(10); 
  }

}
