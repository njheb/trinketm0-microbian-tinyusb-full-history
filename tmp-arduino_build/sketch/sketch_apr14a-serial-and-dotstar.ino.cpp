#include <Arduino.h>
#line 1 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"

#include <Adafruit_TinyUSB.h>

#define DEMO_DOT_STAR
#ifdef DEMO_DOT_STAR
#include <Adafruit_DotStar.h>

#define NUM_PIXELS 1

Adafruit_DotStar strip = Adafruit_DotStar(NUM_PIXELS, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);
#endif

int led = LED_BUILTIN;

#line 15 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void setup();
#line 26 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void demo_setup(void);
#line 31 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void demo_loop(void);
#line 39 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void force_bootloader(void);
#line 65 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void loop();
#line 90 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void rainbow(int wait, long firstPixelHue);
#line 15 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"
void setup() {
  // put your setup code here, to run once:
#ifdef DEMO_DOT_STAR
  strip.begin();
  strip.setBrightness(80);
  strip.show();  // Turn all LEDs off ASAP
#endif
  Serial.begin(9600);
  pinMode(led, OUTPUT);
}

void demo_setup(void)
{
  pinMode(led, OUTPUT);
}

void demo_loop(void)
{
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led,LOW);
  delay(250);
}

void force_bootloader(void)
{
#if defined(__SAMD51__)
#define BOOT_DOUBLE_TAP_ADDRESS ((volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4))
#else
#define BOOT_DOUBLE_TAP_ADDRESS ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
#endif

//#define BOOT_DOUBLE_TAP_ADDRESS           (HSRAM_ADDR + HSRAM_SIZE - 4)
#define BOOT_DOUBLE_TAP_DATA              (*((volatile uint32_t *)BOOT_DOUBLE_TAP_ADDRESS))
#define DOUBLE_TAP_MAGIC                  0xf01669efUL

    /*
         these values are used to call the bootloader on demand
         BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
         NVIC_SystemReset();      // processor software reset
    A uf2 file can be dragged to the BOOT drive that shows on the PC
     */
     BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
     NVIC_SystemReset();      // processor software reset

}

long PixelHue = 0;
int  count = 10000;

void loop() {
  // put your main code here, to run repeatedly:
  count--;
  if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding to microbian port for first test recovery
  {
     force_bootloader();  
  }
  digitalWrite(led, HIGH);
//  delay(1000);
  Serial.println(count);
  digitalWrite(led, LOW);
//  delay(1000);
//  Serial.print("DEF\n");
//  strip.setPixelColor(0, 64, 0, 0); strip.show(); delay(1000); //red
//  strip.setPixelColor(0, 0, 64, 0); strip.show(); delay(1000); //green
//  strip.setPixelColor(0, 0, 0, 64); strip.show(); delay(1000); //blue
#ifdef DEMO_DOT_STAR
  rainbow(10, PixelHue);
  PixelHue += 256;
  if (PixelHue >= 5*65536) PixelHue = 0;
#endif  

}
#ifdef DEMO_DOT_STAR
// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait, long firstPixelHue) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
//  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
//  }
}
#endif

