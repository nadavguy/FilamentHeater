#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char buff[10];
String valueString = "";
uint8_t i = 0;
uint8_t j = 0;
uint8_t k = 0;

void drawbitmap(uint8_t XdisplayPos, uint8_t YdisplayPos, uint8_t bmpWidth, uint8_t bmpHeight, const unsigned char bmpToDisplay[])
{
  //display.clearDisplay();

  display.drawBitmap( XdisplayPos, YdisplayPos, bmpToDisplay, bmpWidth, bmpHeight, 1);
  display.display();
}
// XdisplayPos, in digits
// LineNumber, 0 - 3
void drawText(uint8_t XdisplayPos, uint8_t LineNumber, String TextToDisplay)
{
  //display.clearDisplay();

  display.setTextSize(1);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);     // Draw white text
  display.setCursor(XdisplayPos*6, (LineNumber - 1)*8); // Start at top-left corner
  display.print((TextToDisplay));

  //   display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  //   display.println(3.141592);

  //   display.setTextSize(2);             // Draw 2X-scale text
  //   display.setTextColor(SSD1306_WHITE);
  //   display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  //delay(2000);
}

void ClearScreen(void)
{
  display.clearDisplay();
  display.display();
}

// XdisplayPos, in digits
// LineNumber, 0 - 3
// NumberOfDigits, digits to clear
void ClearDigits(uint8_t XdisplayPos, uint8_t LineNumber, uint8_t NumberOfDigits)
{
  display.setTextSize(1);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_BLACK);     // Draw white text
  // Start at top-left corner
  for (i = 0; i < NumberOfDigits; i++)
  {
    for (j = 0; j < 8; j++)
    {
      for (k = 0; k < 6; k++)
      {
        display.drawPixel(XdisplayPos*6 + k + i*6, j + (LineNumber -1) * 8, SSD1306_BLACK);
      }
    }
  }
  //display.println(" ");
  display.display();
}

void ClearPixels(uint8_t XdisplayPos, uint8_t YdisplayPos, uint8_t NumberOfPixelsX, uint8_t NumberOfPixelsY)
{
  display.setTextSize(1);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_BLACK);     // Draw white text
  // Start at top-left corner

  for (j = 0; j < NumberOfPixelsX; j++)
  {
    for (k = 0; k < NumberOfPixelsY; k++)
    {
      display.drawPixel(XdisplayPos  + j , YdisplayPos  + k, SSD1306_BLACK);
    }
  }

  //display.println(" ");
  display.display();
}

// XdisplayPos, in digits
// LineNumber, 1 - 4
// NumberToDisplay, must be in float 2 digits percision
void drawNumber(uint8_t XdisplayPos, uint8_t LineNumber, float NumberToDisplay)
{
  //display.clearDisplay();
  memset(buff, 0, sizeof(buff));
  valueString = "";
  dtostrf(NumberToDisplay, 4, 2, buff);
  valueString += buff;
  display.setTextSize(1);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);     // Draw white text
  display.setCursor(XdisplayPos * 6, (LineNumber - 1)*8); // Start at top-left corner
  display.print((valueString));

  //   display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  //   display.println(3.141592);

  //   display.setTextSize(2);             // Draw 2X-scale text
  //   display.setTextColor(SSD1306_WHITE);
  //   display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  //delay(2000);
}
