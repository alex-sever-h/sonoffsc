#ifndef TASKRGB_H__
#define TASKRGB_H__

#ifdef WSFX
#include <WS2812FX.h>
#endif

#include <TaskPeriodic.h>

#define RGB_COLOR               0x0000FF
#define RGB_EFFECT              FX_MODE_RAINBOW
#define RGB_SPEED               255
#define RGB_BRIGHTNESS          255


/*
  This is an example of how simple driving a Neopixel can be
  This code is optimized for understandability and changability rather than raw speed
  More info at http://wp.josh.com/2014/05/11/ws2812-neopixels-made-easy/
*/

// Change this to be at least as long as your pixel string (too long will work fine, just be a little slower)

#define PIXELS 18 //96*11  // Number of pixels in the string

// These values depend on which pin your string is connected to and what board you are using
// More info on how to find these at http://www.arduino.cc/en/Reference/PortManipulation

// These values are for the pin that connects to the Data Input pin on the LED strip. They correspond to...

// Arduino Yun:     Digital Pin 8
// DueMilinove/UNO: Digital Pin 12
// Arduino MeagL    PWM Pin 4

// You'll need to look up the port/bit combination for other boards.

// Note that you could also include the DigitalWriteFast header file to not need to to this lookup.

#define PIXEL_PORT  PORTB  // Port of the pin the pixels are connected to
#define PIXEL_DDR   DDRB   // Port of the pin the pixels are connected to
#define PIXEL_BIT   3      // Bit of the pin the pixels are connected to

// These are the timing constraints taken mostly from the WS2812 datasheets
// These are chosen to be conservative and avoid problems rather than for maximum throughput


#define T1H  900    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

#define RES 6000    // Width of the low gap between bits to cause a frame to latch

// Here are some convience defines for using nanoseconds specs to generate actual CPU delays

#define NS_PER_SEC (1000000000L)          // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (F_CPU)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )

#define DELAY_CYCLES(n) ( ((n)>0) ? __builtin_avr_delay_cycles( n ) : __builtin_avr_delay_cycles( 0 ) ) // Make sure we never have a delay less than zero
// Actually send a bit to the string. We must to drop to asm to enusre that the complier does
// not reorder things and make it so the delay happens in the wrong place.
#if 0
void sendBit( bool bitVal ) {

  if ( bitVal ) {      // 1-bit

    bitSet( PIXEL_PORT , PIXEL_BIT );

    sei();
    DELAY_CYCLES( NS_TO_CYCLES( T1H ) - 2 ); // 1-bit width less overhead for the actual bit setting
    cli();

    // Note that this delay could be longer and everything would still work
    bitClear( PIXEL_PORT , PIXEL_BIT );

    DELAY_CYCLES( NS_TO_CYCLES( T1L ) - 10 ); // 1-bit gap less the overhead of the loop

  } else {             // 0-bit

    //cli();                                       // We need to protect this bit from being made wider by an interrupt

    bitSet( PIXEL_PORT , PIXEL_BIT );

    DELAY_CYCLES( NS_TO_CYCLES( T0H ) - 2 ); // 0-bit width less overhead
    // **************************************************************************
    // This line is really the only tight goldilocks timing in the whole program!
    // **************************************************************************
    bitClear( PIXEL_PORT , PIXEL_BIT );

    //sei();

    DELAY_CYCLES( NS_TO_CYCLES( T0L ) - 10 ); // 0-bit gap less overhead of the loop

  }

  // Note that the inter-bit gap can be as long as you want as long as it doesn't exceed the 5us reset timeout (which is A long time)
  // Here I have been generous and not tried to squeeze the gap tight but instead erred on the side of lots of extra time.
  // This has thenice side effect of avoid glitches on very long strings becuase

}
#else
inline void sendBit( bool bitVal ) {

  if (  bitVal ) {				// 0 bit
    //cli();

    asm volatile (
        "sbi %[port], %[bit] \n\t"				// Set the output bit
        ".rept %[onCycles] \n\t"                                // Execute NOPs to delay exactly the specified number of cycles
        "nop \n\t"
        ".endr \n\t"
        "cbi %[port], %[bit] \n\t"                              // Clear the output bit
        ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
        "nop \n\t"
        ".endr \n\t"
        ::
         [port]		"I" (_SFR_IO_ADDR(PIXEL_PORT)),
         [bit]		"I" (PIXEL_BIT),
         [onCycles]	"I" (NS_TO_CYCLES(T1H) - 2),		// 1-bit width less overhead  for the actual bit setting, note that this delay could be longer and everything would still work
         [offCycles] 	"I" (NS_TO_CYCLES(T1L) - 2)			// Minimum interbit delay. Note that we probably don't need this at all since the loop overhead will be enough, but here for correctness

                  );
    //sei();

  } else {					// 1 bit

    // **************************************************************************
    // This line is really the only tight goldilocks timing in the whole program!
    // **************************************************************************
    //cli();

    asm volatile (
        "cli \n\t"
        "sbi %[port], %[bit] \n\t" // Set the output bit
        ".rept %[onCycles] \n\t"   // Now timing actually matters. The 0-bit must be long enough to be detected but not too long or it will be a 1-bit
        "nop \n\t"                 // Execute NOPs to delay exactly the specified number of cycles
        ".endr \n\t"
        "cbi %[port], %[bit] \n\t" // Clear the output bit
        "sei \n\t"
        ".rept %[offCycles] \n\t"  // Execute NOPs to delay exactly the specified number of cycles
        "nop \n\t"
        ".endr \n\t"
        ::
         [port]		"I" (_SFR_IO_ADDR(PIXEL_PORT)),
         [bit]		"I" (PIXEL_BIT),
         [onCycles]	"I" (NS_TO_CYCLES(T0H) - 2),
         [offCycles]	"I" (NS_TO_CYCLES(T0L) - 2)

                  );
  }

  // Note that the inter-bit gap can be as long as you want as long as it doesn't exceed the 5us reset timeout (which is A long time)
  // Here I have been generous and not tried to squeeze the gap tight but instead erred on the side of lots of extra time.
  // This has thenice side effect of avoid glitches on very long strings becuase
}
#endif

inline void sendByte( unsigned char byte ) {

  for( unsigned char bit = 0 ; bit < 8 ; bit++ ) {

    sendBit( bitRead( byte , 7 ) );                // Neopixel wants bit in highest-to-lowest order
    // so send highest bit (bit #7 in an 8-bit byte since they start at 0)
    byte <<= 1;                                    // and then shift left so bit 6 moves into 7, 5 moves into 6, etc

  }
}

/*
  The following three functions are the public API:

  ledSetup() - set up the pin that is connected to the string. Call once at the begining of the program.
  sendPixel( r g , b ) - send a single pixel to the string. Call this once for each pixel in a frame.
  show() - show the recently sent pixel on the LEDs . Call once per frame.

*/


// Set the specified pin up as digital out

void ledsetup() {

  bitSet( PIXEL_DDR , PIXEL_BIT );

}

inline void sendPixel( unsigned char r, unsigned char g , unsigned char b )  {

  sendByte(g);          // Neopixel wants colors in green then red then blue order
  sendByte(r);
  sendByte(b);

}


// Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame

void show() {
  _delay_us( (RES / 1000UL) + 1);				// Round up since the delay must be _at_least_ this long (too short might not work, too long not a problem)
}


/*
  That is the whole API. What follows are some demo functions rewriten from the AdaFruit strandtest code...

  https://github.com/adafruit/Adafruit_NeoPixel/blob/master/examples/strandtest/strandtest.ino

  Note that we always turn off interrupts while we are sending pixels becuase an interupt
  could happen just when we were in the middle of somehting time sensitive.

  If we wanted to minimize the time interrupts were off, we could instead
  could get away with only turning off interrupts just for the very brief moment
  when we are actually sending a 0 bit (~1us), as long as we were sure that the total time
  taken by any interrupts + the time in our pixel generation code never exceeded the reset time (5us).

*/


// Display a single color on the whole string

void showColor( unsigned char r , unsigned char g , unsigned char b ) {

  //cli();
  for( int p=0; p<PIXELS; p++ ) {
    sendPixel( r , g , b );
  }
  //sei();
  show();

}

// Fill the dots one after the other with a color
// rewrite to lift the compare out of the loop
void colorWipe(unsigned char r , unsigned char g, unsigned char b, unsigned  char wait ) {
  for(unsigned int i=0; i<PIXELS; i+= (PIXELS/60) ) {

    //cli();
    unsigned int p=0;

    while (p++<=i) {
      sendPixel(r,g,b);
    }

    while (p++<=PIXELS) {
      sendPixel(0,0,0);

    }

    //sei();
    show();
    delay(wait);
  }
}


class TaskRGB : public TaskPeriodic {
  int pin_;
  int ledCount_;

#ifdef WSFX
  // Parameter 1 = number of pixels in strip
  // Parameter 2 = pin number (most are valid)
  // Parameter 3 = pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  WS2812FX ws2812fx;
#endif
 public:
  TaskRGB(int pin, int ledCount) :TaskPeriodic(100),
                                  pin_(pin),
                                  ledCount_(ledCount)
#ifdef WSFX
                                 , ws2812fx(ledCount, pin, NEO_GRB + NEO_KHZ800)
#endif
  {
  }

  void start(void) {
#ifdef WSFX
    // Neopixel setup and start animation
    ws2812fx.init();
    ws2812fx.setBrightness(RGB_BRIGHTNESS);
    ws2812fx.setSpeed(RGB_SPEED);
    ws2812fx.setColor(RGB_COLOR);
    ws2812fx.setMode(RGB_EFFECT);
    ws2812fx.start();
#else
    ledsetup();
#endif
  }

  unsigned char r, g, b;

  bool work(void) {
    g = 255;
    //cli();
    showColor(r, g, b);
    //sei();

    //r+=10;
    //g+=20;
    //b-=10;

    Serial.print(" r ");
    Serial.print(  r  );
    Serial.print(" g ");
    Serial.print(  g  );
    Serial.print(" b ");
    Serial.print(  b  );
    Serial.println();

    return true;
  }


};



#endif
