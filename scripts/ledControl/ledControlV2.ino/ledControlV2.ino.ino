#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define INT_LED_PIN   10
#define INT_LED_COUNT 60 //pour vrai je sais pas faudrait compter

// Declare our NeoPixel strip object:
Adafruit_NeoPixel intStrip(INT_LED_COUNT, INT_LED_PIN, NEO_GRB + NEO_KHZ800);

bool touch = false;
bool altern = false;

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

  Serial.begin(9600);
}


void loop() {

  String incomingMsg;
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingMsg = Serial.readString();
  }

  if(incomingMsg == "t\n")
  {
    touch = true;
  }
  else if(incomingMsg == "s\n")
  {
    touch = false;
  }

  startInternalLight();
  delay(100);
}

void startInternalLight() {
  if(touch)
  {
    if(altern)
    {
      for(int i=8; i<17; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(255,0,0));                                                      
      }
      for(int i=26; i<33; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,220));                                                      
      }
      for(int i=1; i<8; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,0));                                                      
      }
      for(int i=18; i<25; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,0));  
      }
      /*for(int i=0; i<32; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,90));                                                      
      }*/
      intStrip.show(); 
      delay(200);
      Serial.println("x");                                           
    }
    else
    {
      for(int i=8; i<17; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,0));                                                      
      }
      for(int i=26; i<33; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,0));                                                      
      }
      for(int i=1; i<8; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(255,0,0));                                                      
      }
      for(int i=18; i<25; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(0,0,220));  
      }
      /*  
            for(int i=0; i<32; i++) { 
        intStrip.setPixelColor(i, intStrip.Color(200,0,0));                                                      
      }*/
      intStrip.show(); 
      delay(200);
      Serial.println("y");        
    }
    altern = !altern;
  }
  else
  {
    for(int i=0; i<intStrip.numPixels(); i++) { 
      intStrip.setPixelColor(i, intStrip.Color(0,0,0));      
      intStrip.show();                                                
    }
  }
}
