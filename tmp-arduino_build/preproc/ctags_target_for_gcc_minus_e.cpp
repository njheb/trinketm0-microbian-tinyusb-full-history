# 1 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino"

# 3 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino" 2



# 7 "/home/nick/Arduino/sketch_apr14a-serial-and-dotstar/sketch_apr14a-serial-and-dotstar.ino" 2



Adafruit_DotStar strip = Adafruit_DotStar(1, (7u), (8u), (2 | (1 << 2) | (0 << 4)) /*|< Transmit as B,G,R*/);


int led = (13u);

void setup() {
  // put your setup code here, to run once:

  strip.begin();
  strip.setBrightness(80);
  strip.show(); // Turn all LEDs off ASAP

  Serial.begin(9600);
  pinMode(led, (0x1));
}

void demo_setup(void)
{
  pinMode(led, (0x1));
}

void demo_loop(void)
{
  digitalWrite(led, (0x1));
  delay(500);
  digitalWrite(led,(0x0));
  delay(250);
}

void force_bootloader(void)
{






//#define BOOT_DOUBLE_TAP_ADDRESS           (HSRAM_ADDR + HSRAM_SIZE - 4)



    /*
         these values are used to call the bootloader on demand
         BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
         NVIC_SystemReset();      // processor software reset
    A uf2 file can be dragged to the BOOT drive that shows on the PC
     */
     (*((volatile uint32_t *)((volatile uint32_t *)((0x20000000u) /**< HMCRAMC0 base address */ + 0x8000UL /* 32 kB */ - 4)))) = 0xf01669efUL;
     __NVIC_SystemReset(); // processor software reset

}

long PixelHue = 0;
int count = 10000;

void loop() {
  // put your main code here, to run repeatedly:
  count--;
  if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding to microbian port for first test recovery
  {
     force_bootloader();
  }
  digitalWrite(led, (0x1));
//  delay(1000);
  Serial.println(count);
  digitalWrite(led, (0x0));
//  delay(1000);
//  Serial.print("DEF\n");
//  strip.setPixelColor(0, 64, 0, 0); strip.show(); delay(1000); //red
//  strip.setPixelColor(0, 0, 64, 0); strip.show(); delay(1000); //green
//  strip.setPixelColor(0, 0, 0, 64); strip.show(); delay(1000); //blue

  rainbow(10, PixelHue);
  PixelHue += 256;
  if (PixelHue >= 5*65536) PixelHue = 0;


}

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
    delay(wait); // Pause for a moment
//  }
}
