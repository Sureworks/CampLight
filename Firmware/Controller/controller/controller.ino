/*********************************************************************
 Based on "controller" by Adafruit.

 Modified by: Darren Odom
 Date 6/4/16

 Changelog: 6/4/16: Initial adaptation from controller to enable
 buttons, debouncing of buttons, 
*********************************************************************/


#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif
#if !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266) && !defined(ARDUINO_ARCH_STM32F2)
 #include <util/delay.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include <Bounce2.h>
//#include <Adafruit_NeoPixel.h>  <--- this is a slow library!
#include "FastLED.h"   //  <--- this one is fast.

FASTLED_USING_NAMESPACE

//FIRE:
// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100 
#define COOLING  55

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 120

#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

bool gReverseDirection = false;

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif


//#define CLK_PIN   4



/*========================================================================
 *   PHYSICAL PIN SETUP
 *   
 ========================================================================*/
#define BUTTON1_PIN 15
#define BUTTON2_PIN 14
#define BUTTON3_PIN 17
#define BUTTON4_PIN 16
#define DATA_PIN    5 //Neopixel pin
#define NUM_LEDS    4

#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define  R_PIN  6  // Red LED
#define  G_PIN  10  // Green LED
#define  B_PIN  11  // Blue LED
#define  W_PIN 12  //White LED
#define BUTT_HOLD_THRESHOLD 200
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// function prototypes over in Colorspace.cpp
void init_colors();
CRGB iterate();

// the packet buffer
extern uint8_t packetbuffer[];

 // Instantiate a Bounce objects
Bounce Button1 = Bounce(); 
Bounce Button2 = Bounce(); 
Bounce Button3 = Bounce(); 
Bounce Button4 = Bounce(); 

 // Instantiate a Neopixel strip object
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
CRGB leds[NUM_LEDS];
      

  
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  //delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection 
  while (! ble.isConnected()) {
      delay(500);
  }*/

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  // Setup the button with an internal pull-up :
  pinMode(BUTTON1_PIN,INPUT_PULLUP);
  pinMode(BUTTON2_PIN,INPUT_PULLUP);
  pinMode(BUTTON3_PIN,INPUT_PULLUP); 
  pinMode(BUTTON4_PIN,INPUT_PULLUP);

  //Set up main LED pin as output
  pinMode(R_PIN, OUTPUT);   // sets the pins as output
  pinMode(G_PIN, OUTPUT);  
  pinMode(B_PIN, OUTPUT);
  pinMode(W_PIN, OUTPUT);
  
  // After setting up the button, setup the Bounce instance :
  Button1.attach(BUTTON1_PIN);
  Button1.interval(5); // interval in ms
  Button2.attach(BUTTON2_PIN);
  Button2.interval(5); // interval in ms
  Button3.attach(BUTTON3_PIN);
  Button3.interval(5); // interval in ms
  Button4.attach(BUTTON4_PIN);
  Button4.interval(5); // interval in ms
  
  //Set-up neopixel strip
 
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
//SimplePatternList gPatterns = { off,solid, rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm, Fire2012  };
SimplePatternList gPatterns = { off,solid };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
    
//Storage of button state
boolean butt1;
boolean butt2;
boolean butt3;
boolean butt4;

int butt3_held;
int butt4_held;

int bt_butt_held;
boolean bt_hold_pressed;
uint8_t bt_hold_buttnum;

//Color storage
uint8_t red =0;
uint8_t green =0;
uint8_t blue = 0;

//BrightLED analog output value
uint8_t PWM_value = 0;


void loop(void)
{
   //BTLE button press vars:
  uint8_t buttnum;
  boolean pressed;
  boolean released;
  
  boolean butt1_last= butt1;
  boolean butt2_last= butt2;
  boolean butt3_last= butt3;
  boolean butt4_last= butt4;
  
  boolean colorchange = 0;
   // Update the Bounce instance :
  Button1.update();
  Button2.update();
  Button3.update();
  Button4.update();

  butt1 = !Button1.read();
  butt2 = !Button2.read();
  butt3 = !Button3.read();
  butt4 = !Button4.read();
  
  if(butt1>butt1_last)
    Serial.println("button1 pressed");
  if(butt1<butt1_last){
    Serial.println("button1 released");
    nextPattern();
  }
  if(butt2<butt2_last){
    Serial.println("button2 released");
    prevPattern();
  }
  if(butt3)
    butt3_held++;
    if(butt3_held >= BUTT_HOLD_THRESHOLD)
      PWM_value--;
  if(butt4)
  {
    butt4_held++;
    if(butt4_held >= BUTT_HOLD_THRESHOLD)
      PWM_value++;
  }
    
  if(butt3<butt3_last){
    Serial.println("button3 released");
    if(butt3_held < BUTT_HOLD_THRESHOLD)
      PWM_value =0;
    butt3_held = 0;  //return the hold counter to 0.
  }
  if(butt4<butt4_last){
    Serial.println("button3 released");
    if(butt4_held < BUTT_HOLD_THRESHOLD)
      PWM_value =255;
    butt4_held = 0;  //return the hold counter to 0.
  }

 
 if(bt_hold_pressed)
 {
  bt_butt_held++;
  
  if(bt_butt_held >= BUTT_HOLD_THRESHOLD)
    if(bt_hold_buttnum == 5)
      PWM_value++;
      
  if(bt_butt_held >= BUTT_HOLD_THRESHOLD)
    if(bt_hold_buttnum == 6)
      PWM_value--;
 }
   // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  //Call the PWM output "analog Write"
  if(PWM_value > 255) PWM_value=255;
  if(PWM_value <0) PWM_value = 0;
  analogWrite(W_PIN, PWM_value);
  analogWrite(R_PIN, red);
  analogWrite(G_PIN, green);
  analogWrite(B_PIN, blue);
  //analogWrite(PWM_PIN, PWM_value);
  //digitalWrite(PWM_PIN,PWM_value);
  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 200 ) { gHue++; } // slowly cycle the "base color" through the rainbow
 
  
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);
  

  // Color
  if (packetbuffer[1] == 'C') {
    red = packetbuffer[2];
    green = packetbuffer[3];
    blue = packetbuffer[4];
    colorchange = 1;
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    buttnum = packetbuffer[2] - '0';
    bt_hold_buttnum = buttnum;
    pressed = packetbuffer[3] - '0';
    released = !pressed;
    bt_hold_pressed = pressed;  //Store the same press state in a more global var.
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }


 
  //cycle neopixel show based on BT button press
  if  (pressed && buttnum <=4)  {
     gCurrentPatternNumber = buttnum;
    }
  if  (pressed && buttnum ==8)  {  //Right
     nextPattern();
    }
  if  (pressed && buttnum ==7)  { //Left
     prevPattern();
    }
  if  (released && buttnum ==6)   { //Down
     if(bt_butt_held < BUTT_HOLD_THRESHOLD)
     {
      PWM_value = 0;
     }
     bt_butt_held = 0;
  }
  if  (released && buttnum ==5)  {  //Up
    if(bt_butt_held < BUTT_HOLD_THRESHOLD)
     {
      PWM_value = 255;
     }
     bt_butt_held = 0;
   }

  if (colorchange)
    gCurrentPatternNumber = 0;
    
}


#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
 
}
void prevPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber - 1) % ARRAY_SIZE( gPatterns);
  
}

void off()
{

   red= 0;
   green = 0;
   blue = 0;
   for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] =  leds[i].setRGB( red, green, blue);
   }
}

void solid()
{
    /*convert hue to RGB*/
   
   CRGB rgb =iterate(); //Move through the colors (See Colorspace.cpp)
   red= rgb.red;
   green = rgb.green;
   blue = rgb.blue;
   for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] =  leds[i].setRGB( red, green, blue);
   }
   
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void Fire2012()
{
// Array of temperature readings at each simulation cell
  static byte heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }
}

