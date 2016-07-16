/*********************************************************************
 Based on "controller" by Adafruit.

 Modified by: Darren Odom
 Date 6/4/16

 Changelog: 6/4/16: Initial adaptation from controller to enable
 buttons, debouncing of buttons, 
 7/15/16: Added RGBW 
 
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

// the packet buffer
extern uint8_t packetbuffer[];
/*
RGB LED - Automatic Smooth Color Cycling

Marco Colli
April 2012

Uses the properties of the RGB Colour Cube
The RGB colour space can be viewed as a cube of colour. If we assume a cube of dimension 1, then the 
coordinates of the vertices for the cubve will range from (0,0,0) to (1,1,1) (all black to all white).
The transitions between each vertex will be a smooth colour flow and we can exploit this by using the 
path coordinates as the LED transition effect. 
*/
// Output pins for PWM
#include <FastLED.h>
#define DATA_PIN 5
#define NUM_LEDS 4
#define COLOR_ORDER GRB

#define  R_PIN  6  // Red LED
#define  G_PIN  10  // Green LED
#define  B_PIN  11  // Blue LED
#define W_PIN 12  //White LED

// Constants for readability are better than magic numbers
// Used to adjust the limits for the LED, especially if it has a lower ON threshold
#define  MIN_RGB_VALUE  30   // no smaller than 0. 
#define  MAX_RGB_VALUE  255  // no bigger than 255.

// Slowing things down we need ...
#define  TRANSITION_DELAY  70   // in milliseconds, between individual light changes
#define  WAIT_DELAY        0  // in milliseconds, at the end of each traverse
//
// Total traversal time is ((MAX_RGB_VALUE - MIN_RGB_VALUE) * TRANSITION_DELAY) + WAIT_DELAY
// eg, ((255-0)*70)+500 = 18350ms = 18.35s

// Structure to contain a 3D coordinate
typedef struct
{
  byte  x, y, z;
} coord;

static coord  v; // the current rgb coordinates (colour) being displayed

/*
Vertices of a cube
      
    C+----------+G
    /|        / |
  B+---------+F |
   | |       |  |    y   
   |D+-------|--+H   ^  7 z
   |/        | /     | /
  A+---------+E      +--->x

*/
const coord vertex[] = 
{
//x  y  z      name
  {0, 0, 0}, // A or 0
  {0, 1, 0}, // B or 1
  {0, 1, 1}, // C or 2
  {0, 0, 1}, // D or 3
  {1, 0, 0}, // E or 4
  {1, 1, 0}, // F or 5
  {1, 1, 1}, // G or 6
  {1, 0, 1}  // H or 7
};

/*
A list of vertex numbers encoded 2 per byte.
Hex digits are used as vertices 0-7 fit nicely (3 bits 000-111) and have the same visual
representation as decimal, so bytes 0x12, 0x34 ... should be interpreted as vertex 1 to 
v2 to v3 to v4 (ie, one continuous path B to C to D to E).
*/
const byte path[] =
{
  0x01, 0x23, 0x76, 0x54, 0x03, 0x21, 0x56, 0x74,  // trace the edges
  0x13, 0x64, 0x16, 0x02, 0x75, 0x24, 0x35, 0x17, 0x25, 0x70,  // do the diagonals
};

#define  MAX_PATH_SIZE  (sizeof(path)/sizeof(path[0]))  // size of the array

CRGB leds[NUM_LEDS];

int bt_butt_held;
boolean bt_hold_pressed;
uint8_t bt_hold_buttnum;

//BrightLED analog output value
uint8_t PWM_value = 0;

void setup()
{
  pinMode(R_PIN, OUTPUT);   // sets the pins as output
  pinMode(G_PIN, OUTPUT);  
  pinMode(B_PIN, OUTPUT);
  pinMode(W_PIN, OUTPUT);
  analogWrite(W_PIN, 0);
  FastLED.addLeds<WS2811, DATA_PIN, GRB>(leds, NUM_LEDS);


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

}

void traverse(int dx, int dy, int dz)
// Move along the colour line from where we are to the next vertex of the cube.
// The transition is achieved by applying the 'delta' value to the coordinate.
// By definition all the coordinates will complete the transition at the same 
// time as we only have one loop index.
{
  if ((dx == 0) && (dy == 0) && (dz == 0))   // no point looping if we are staying in the same spot!
    return;
    
  for (int i = 0; i < MAX_RGB_VALUE-MIN_RGB_VALUE; i++, v.x += dx, v.y += dy, v.z += dz)
  {
    // set the colour in the LED
    analogWrite(R_PIN, v.x);
    analogWrite(G_PIN, v.y);
    analogWrite(B_PIN, v.z);
    for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(v.x,v.y,v.z);
      }
    // Display the colors we just set on the actual LEDs
    FastLED.show();
    
    delay(TRANSITION_DELAY);  // wait fot the transition delay
  }

  delay(WAIT_DELAY);          // give it an extra rest at the end of the traverse
}

void loop()
{
  int    v1, v2=0;    // the new vertex and the previous one
 
   //BTLE button press vars:
  uint8_t buttnum;
  boolean pressed;
  boolean released;

  
  // initialise the place we start from as the first vertex in the array
  v.x = (vertex[v2].x ? MAX_RGB_VALUE : MIN_RGB_VALUE);
  v.y = (vertex[v2].y ? MAX_RGB_VALUE : MIN_RGB_VALUE);
  v.z = (vertex[v2].z ? MAX_RGB_VALUE : MIN_RGB_VALUE);

  // Now just loop through the path, traversing from one point to the next
  for (int i = 0; i < 2*MAX_PATH_SIZE; i++)
  {
    // !! loop index is double what the path index is as it is a nybble index !!
    v1 = v2;
    if (i&1)  // odd number is the second element and ...
      v2 = path[i>>1] & 0xf;  // ... the bottom nybble (index /2) or ...
    else      // ... even number is the first element and ...
      v2 = path[i>>1] >> 4;  // ... the top nybble
      
    traverse(vertex[v2].x-vertex[v1].x, 
             vertex[v2].y-vertex[v1].y, 
             vertex[v2].z-vertex[v1].z);
  }
   /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

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
  //cycle neopixel show based on BT button press
  if  (pressed && buttnum <=4)  {
     //gCurrentPatternNumber = buttnum;
    }
  if  (pressed && buttnum ==8)  {  //Right
     //nextPattern();
    }
  if  (pressed && buttnum ==7)  { //Left
     //prevPattern();
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
}

