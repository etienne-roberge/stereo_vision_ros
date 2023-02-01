#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define INT_LED_PIN    10
#define EXT_LED_PIN    11

#define INT_LED_COUNT 60 //pour vrai je sais pas faudrait compter
#define EXT_LED_COUNT 60 //pour vrai je sais pas faudrait compter

// Declare our NeoPixel strip object:
Adafruit_NeoPixel intStrip(INT_LED_COUNT, INT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel extStrip(EXT_LED_COUNT, EXT_LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  intStrip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  intStrip.show();            // Turn OFF all pixels ASAP
  intStrip.setBrightness(200); // Set BRIGHTNESS to about 1/5 (max = 255)
  extStrip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  extStrip.show();            // Turn OFF all pixels ASAP
  extStrip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  Serial.begin(9600);
}


void loop() {

  String incomingMsg;
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingMsg = Serial.readString();
  }

  if(incomingMsg == "startTouch\n")
  {
    startInternalLight(true);
    startExternalLight(false);
  }
  else if(incomingMsg == "startVision\n")
  {
    startInternalLight(false);
    startExternalLight(true);
  }
  else if(incomingMsg == "stop\n")
  {
    startInternalLight(false);
    startExternalLight(false);
  }
  delay(100);
}

void startInternalLight(bool start) {
  if(start)
  {
    for(int i=0; i<10; i++) { 
      intStrip.setPixelColor(i, intStrip.Color(255,0,0));      
      intStrip.show();                                                 
    }
    for(int i=10; i<20; i++) { 
      intStrip.setPixelColor(i, intStrip.Color(0,255,0));      
      intStrip.show();                                                 
    }
    for(int i=20; i<30; i++) { 
      intStrip.setPixelColor(i, intStrip.Color(0,0,255));      
      intStrip.show();                                                 
    }
  }
  else
  {
    for(int i=0; i<intStrip.numPixels(); i++) { 
      intStrip.setPixelColor(i, intStrip.Color(0,0,0));      
      intStrip.show();                                                
    }
  }
}

void startExternalLight(bool start) {
  if(start)
  {
    for(int i=0; i<extStrip.numPixels(); i++) { 
      extStrip.setPixelColor(i, extStrip.Color(255,255,255));      
      extStrip.show();                                                 
    }
  }
  else
  {
    for(int i=0; i<extStrip.numPixels(); i++) { 
      extStrip.setPixelColor(i, extStrip.Color(0,0,0));      
      extStrip.show();                                                
    }
  }
}
