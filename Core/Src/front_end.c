/*
 * front_end.c
 *
 *  Created on: Nov 27, 2024
 *      Author: buith
 */
#include "front_end.h"

#define min(a,b) (((a)<(b))?(a):(b))

void demoLCD(int i)
{
 lcdSetOrientation(i%4);

 uint32_t t = testFillScreen();
 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", t);
 HAL_Delay(2000);

 t = HAL_GetTick();
 lcdTest();
 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", HAL_GetTick() - t);
 HAL_Delay(2000);

 t = testText();
 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", t);
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testLines(COLOR_CYAN));
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testFastLines(COLOR_RED, COLOR_BLUE));
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testRects(COLOR_GREEN));
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testFilledRects(COLOR_YELLOW, COLOR_MAGENTA));
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testFilledCircles(10, COLOR_MAGENTA));
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testCircles(10, COLOR_WHITE));
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testTriangles());
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testFilledTriangles());
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testRoundRects());
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testFilledRoundRects());
 HAL_Delay(2000);

 lcdSetTextFont(&Font16);
 lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("Time: %4lu ms", testDrawImage());
 HAL_Delay(2000);
}

unsigned long testFillScreen()
{
 unsigned long start = HAL_GetTick(), t = 0;
 lcdFillRGB(COLOR_BLACK);
 t += HAL_GetTick() - start;
 lcdSetCursor(0, 0);
 lcdSetTextFont(&Font24);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("BLACK");
 HAL_Delay(1000);
 start = HAL_GetTick();
 lcdFillRGB(COLOR_RED);
 t += HAL_GetTick() - start;
 lcdSetCursor(0, 0);
 lcdSetTextFont(&Font24);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("RED");
 HAL_Delay(1000);
 start = HAL_GetTick();
 lcdFillRGB(COLOR_GREEN);
 t += HAL_GetTick() - start;
 lcdSetCursor(0, 0);
 lcdSetTextFont(&Font24);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("GREEN");
 HAL_Delay(1000);
 start = HAL_GetTick();
 lcdFillRGB(COLOR_BLUE);
 t += HAL_GetTick() - start;
 lcdSetCursor(0, 0);
 lcdSetTextFont(&Font24);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdPrintf("BLUE");
 HAL_Delay(1000);
 start = HAL_GetTick();
 lcdFillRGB(COLOR_BLACK);
 return t += HAL_GetTick() - start;
}

unsigned long testText()
{
 lcdFillRGB(COLOR_BLACK);
 unsigned long start = HAL_GetTick();
 lcdSetCursor(0, 0);
 lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
 lcdSetTextFont(&Font8);
 lcdPrintf("Hello World!\r\n");
 lcdSetTextColor(COLOR_YELLOW, COLOR_BLACK);
 lcdSetTextFont(&Font12);
 lcdPrintf("%i\r\n", 1234567890);
 lcdSetTextColor(COLOR_RED, COLOR_BLACK);
 lcdSetTextFont(&Font16);
 lcdPrintf("%#X\r\n", 0xDEADBEEF);
 lcdPrintf("\r\n");
 lcdSetTextColor(COLOR_GREEN, COLOR_BLACK);
 lcdSetTextFont(&Font20);
 lcdPrintf("Groop\r\n");
 lcdSetTextFont(&Font12);
 lcdPrintf("I implore thee,\r\n");
 lcdSetTextFont(&Font12);
 lcdPrintf("my foonting turlingdromes.\r\n");
 lcdPrintf("And hooptiously drangle me\r\n");
 lcdPrintf("with crinkly bindlewurdles,\r\n");
 lcdPrintf("Or I will rend thee\r\n");
 lcdPrintf("in the gobberwarts\r\n");
 lcdPrintf("with my blurglecruncheon,\r\n");
 lcdPrintf("see if I don't!\r\n");
 return HAL_GetTick() - start;
}

unsigned long testLines(uint16_t color)
{
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = lcdGetWidth(),
                h = lcdGetHeight();

  lcdFillRGB(COLOR_BLACK);

  x1 = y1 = 0;
  y2 = h - 1;
  start = HAL_GetTick();
  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  t = HAL_GetTick() - start; // fillScreen doesn't count against timing

  HAL_Delay(1000);
  lcdFillRGB(COLOR_BLACK);

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;

  start = HAL_GetTick();

  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  t += HAL_GetTick() - start;

  HAL_Delay(1000);
  lcdFillRGB(COLOR_BLACK);

  x1 = 0;
  y1 = h - 1;
  y2 = 0;
  start = HAL_GetTick();

  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  t += HAL_GetTick() - start;

  HAL_Delay(1000);
  lcdFillRGB(COLOR_BLACK);

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;

  start = HAL_GetTick();

  for(x2 = 0; x2 < w; x2 += 6) lcdDrawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for(y2 = 0; y2 < h; y2 += 6) lcdDrawLine(x1, y1, x2, y2, color);

  return t += HAL_GetTick() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2)
{
  unsigned long start;
  int x, y, w = lcdGetWidth(), h = lcdGetHeight();

  lcdFillRGB(COLOR_BLACK);
  start = HAL_GetTick();
  for(y = 0; y < h; y += 5) lcdDrawHLine(0, w, y, color1);
  for(x = 0; x < w; x += 5) lcdDrawVLine(x, 0, h, color2);

  return HAL_GetTick() - start;
}

unsigned long testRects(uint16_t color)
{
  unsigned long start;
  int n, i, i2,
      cx = lcdGetWidth()  / 2,
      cy = lcdGetHeight() / 2;

  lcdFillRGB(COLOR_BLACK);
  n = min(lcdGetWidth(), lcdGetHeight());
  start = HAL_GetTick();
  for(i = 2; i < n; i += 6)
  {
    i2 = i / 2;
    lcdDrawRect(cx - i2, cy - i2, i, i, color);
  }

  return HAL_GetTick() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2)
{
  unsigned long start, t = 0;
  int n, i, i2,
      cx = lcdGetWidth() / 2 - 1,
      cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  n = min(lcdGetWidth(), lcdGetHeight());

  for(i = n; i > 0; i -= 6)
  {
    i2 = i / 2;
    start = HAL_GetTick();
    lcdFillRect(cx-i2, cy-i2, i, i, color1);
    t    += HAL_GetTick() - start;
    // Outlines are not included in timing results
    lcdDrawRect(cx-i2, cy-i2, i, i, color1);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color)
{
  unsigned long start;
  int x, y, w = lcdGetWidth(), h = lcdGetHeight(), r2 = radius * 2;

  lcdFillRGB(COLOR_BLACK);
  start = HAL_GetTick();
  for(x = radius; x < w; x += r2)
  {
    for(y = radius; y < h; y += r2)
    {
      lcdFillCircle(x, y, radius, color);
    }
  }

  return HAL_GetTick() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color)
{
  unsigned long start;
  int x, y, r2 = radius * 2,
      w = lcdGetWidth()  + radius,
      h = lcdGetHeight() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = HAL_GetTick();
  for(x = 0; x < w; x += r2)
  {
    for(y = 0; y < h; y += r2)
    {
      lcdDrawCircle(x, y, radius, color);
    }
  }

  return HAL_GetTick() - start;
}

unsigned long testTriangles()
{
  unsigned long start;
  int n, i, cx = lcdGetWidth() / 2 - 1,
            cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  n = min(cx, cy);
  start = HAL_GetTick();
  for(i = 0; i < n; i += 5)
  {
    lcdDrawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      lcdColor565(i, i, i));
  }

  return HAL_GetTick() - start;
}

unsigned long testFilledTriangles()
{
  unsigned long start, t = 0;
  int i, cx = lcdGetWidth() / 2 - 1,
         cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  for(i = min(cx, cy); i > 10; i -= 5)
  {
    start = HAL_GetTick();
    lcdFillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, lcdColor565(0, i*10, i*10));
    t += HAL_GetTick() - start;
    lcdFillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, lcdColor565(i*10, i*10, 0));
  }

  return t;
}

unsigned long testRoundRects()
{
  unsigned long start;
  int w, i, i2,
      cx = lcdGetWidth() / 2 - 1,
      cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  w = lcdGetWidth(), lcdGetHeight();
  start = HAL_GetTick();
  for(i = 0; i < w; i += 6)
  {
    i2 = i / 2;
    lcdDrawRoundRect(cx-i2, cy-i2, i, i, i/2, lcdColor565(i, 0, 0));
    HAL_Delay(100);
  }

  return HAL_GetTick() - start;
}

unsigned long testFilledRoundRects()
{
  unsigned long start;
  int i, i2,
      cx = lcdGetWidth()  / 2 - 1,
      cy = lcdGetHeight() / 2 - 1;

  lcdFillRGB(COLOR_BLACK);
  start = HAL_GetTick();
  for(i = min(lcdGetWidth(), lcdGetHeight()); i > 20; i -=6 )
  {
    i2 = i / 2;
    lcdFillRoundRect(cx - i2, cy - i2, i, i, i / 8, lcdColor565(0, i, 0));
  }

  return HAL_GetTick() - start;
}

unsigned long testDrawImage()
{
 unsigned long start;

 start = HAL_GetTick();

 if (lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE || lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE_MIRROR)
 {
  lcdDrawImage((lcdGetWidth() - bmBackGround.xSize)/2, 0, &bmBackGround);
 }
 else
 {
  lcdDrawImage(0, (lcdGetHeight() - bmBackGround.ySize) / 2, &bmBackGround);
 }
 return HAL_GetTick() - start;
}

void DrawCloud(int x, int y) {
	lcdDrawImage(x, y, &bmicon3);
}

void TextTitle(int16_t x, int16_t y){
	lcdSetCursor(x, y);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdSetTextFont(&Font20);
	lcdPrintf("Weather Report\r\n");

}
void TableMain (int16_t x, int16_t y ,int temperature , int huminity )
{
	lcdDrawRect(x, y, 140 , 50 , COLOR_WHITE);
	lcdDrawLine(x, y + 15, x + 139 , y + 15, COLOR_WHITE);
	lcdDrawLine(x + 70, y , x + 70, y + 49, COLOR_WHITE);

	lcdSetCursor(x + 7, y + 4);
    lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
    lcdSetTextFont(&Font8);
    lcdPrintf("Temperature\r\n");

    lcdSetCursor(x + 85 , y + 4);
    lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
    lcdSetTextFont(&Font8);
    lcdPrintf("Huminity\r\n");

    lcdSetCursor(x + 4 , y + 25);
    lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
    lcdSetTextFont(&Font20);
    if (10 < temperature)
    {
    	lcdSetCursor(x + 7 , y + 25);
    	lcdPrintf("%d C\r\n" , temperature);
		lcdDrawCircle( x + 40 , y + 28 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 40 , y + 28 , 2, COLOR_WHITE);
    }
    else if ( 0 <  temperature)
    {
        lcdSetCursor(x + 14 , y + 25);
    	lcdPrintf("%d C\r\n" , temperature);
    	lcdDrawCircle( x + 35 , y + 28 , 3, COLOR_WHITE);
    	lcdDrawCircle( x + 35 , y + 28 , 2, COLOR_WHITE);
    }
    else if (-10 < temperature )
    {
    	lcdSetCursor(x + 7 , y + 25);
    	lcdPrintf("%d C\r\n" , temperature);
		lcdDrawCircle( x + 40 , y + 28 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 40 , y + 28 , 2, COLOR_WHITE);
    }
    else
    {
    	lcdPrintf("%d" , temperature);
    	lcdSetCursor(x + 53 , y + 25);
    	lcdPrintf("C\r\n");
		lcdDrawCircle( x + 49 , y + 28 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 49 , y + 28 , 2, COLOR_WHITE);
    }

    lcdSetCursor(x + 84 , y + 25);
    lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
    lcdSetTextFont(&Font20);
    if (huminity < 10){
    	lcdSetCursor(x + 91 , y + 25);
    	lcdPrintf("%d/\r\n" , huminity);
		lcdDrawCircle( x + 108 , y + 28 , 2, COLOR_WHITE);
		lcdDrawCircle( x + 116 , y+ 37 , 2, COLOR_WHITE);
    }
    else {
    	lcdPrintf("%d/\r\n" , huminity);
    	lcdDrawCircle( x + 115 , y + 28 , 2, COLOR_WHITE);
    	lcdDrawCircle( x + 123 , y + 37 , 2, COLOR_WHITE);
    }
}
int numDay(int date , int month , int year){
	int sum = 0;
	for (int i = 1 ; i < month ; i++){
		if (i == 2){
			if (CheckYear(year)){
				sum += 29;
			}
			else sum += 28;
		}
		else if (i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i == 12){
			sum += 31;
		}
		else if (i == 4 || i == 6 || i == 9 || i == 11){
			sum += 30;
		}
	}
	sum += date;
	return sum;
}
bool CheckYear(int year){
	if (((year % 4)==0) && ((year % 100)!=0)){
	  return true;
	 }else if ((year % 400)==0){
	  return true;
	 }else {
	  return false;
	 }
}
int FindDate(int day , int month , int year){
	int JMD;
   JMD = (day + ((153 * (month + 12 * ((14 - month) / 12) - 3) + 2) / 5) +
	 (365 * (year + 4800 - ((14 - month) / 12))) +
	 ((year + 4800 - ((14 - month) / 12)) / 4) -
	((year + 4800 - ((14 - month) / 12)) / 100) +
	((year + 4800 - ((14 - month) / 12)) / 400)  - 32045) % 7;
	 return JMD;
}

void lcdDrawThickArc(int x0, int y0, int r, int start_angle, int end_angle, int thickness, uint16_t color) {
    for (int t = 0; t < thickness; t++) {
        int x, y;
        for (int angle = start_angle; angle <= end_angle; angle++) {
            x = x0 + (r + t) * cos(angle * M_PI / 180);
            y = y0 + (r + t) * sin(angle * M_PI / 180);
            lcdDrawPixel(x, y, color); // Giả sử bạn có hàm lcdDrawPixel để vẽ điểm ảnh
        }
    }
}
void DrawClock (int x0, int y0, int r, int start_angle, int end_angle, int thickness){
	lcdDrawThickArc(x0 , y0 ,r , start_angle , -93  , thickness , COLOR_GREEN);
	lcdDrawThickArc(x0 , y0 ,r , -87 , -23  , thickness , COLOR_BLUE);
	lcdDrawThickArc(x0 , y0 ,r , -17 , end_angle  , thickness , COLOR_ORANGE);
}

void CloudRain (int x, int y){
    // Draw the cloud body using rounded rectangles and circles
	lcdFillRoundRect(x, y, 30, 10, 5, COLOR_WHITE);
    lcdFillCircle(x + 24, y + 2, 7, COLOR_WHITE);
    lcdFillCircle(x + 15, y , 9, COLOR_WHITE);

    lcdDrawLine(x + 7, y + 13, x + 5, y + 16, COLOR_BLUE);
    lcdDrawLine(x + 8, y + 13, x + 6, y + 16, COLOR_BLUE);

    lcdDrawLine(x + 13, y + 13, x + 8, y + 19, COLOR_BLUE);
    lcdDrawLine(x + 14, y + 13, x + 9, y + 19, COLOR_BLUE);

    lcdDrawLine(x + 19, y + 13, x + 17, y + 16, COLOR_BLUE);
    lcdDrawLine(x + 20, y + 13, x + 18, y + 16, COLOR_BLUE);

    lcdDrawLine(x + 25, y + 13, x + 20, y + 19, COLOR_BLUE);
    lcdDrawLine(x + 26, y + 13, x + 21, y + 19, COLOR_BLUE);
}
void CloudSun (int x, int y){

	lcdFillCircle(x + 21, y - 5 , 7, COLOR_DARKYELLOW);
    // Draw the cloud body using rounded rectangles and circles
	lcdFillRoundRect(x, y, 30, 10, 5, COLOR_WHITE);
    lcdFillCircle(x + 24, y + 2, 7, COLOR_WHITE);
    lcdFillCircle(x + 15, y , 9, COLOR_WHITE);
}
void CloudMoon (int x, int y){

	lcdFillCircle(x + 10, y - 5 , 7, COLOR_DARKYELLOW);
	lcdFillCircle(x + 7, y - 8 , 4, COLOR_THEME_SKYBLUE_BASE);
    // Draw the cloud body using rounded rectangles and circles
	lcdFillRoundRect(x, y, 30, 10, 5, COLOR_WHITE);
    lcdFillCircle(x + 24, y + 2, 7, COLOR_WHITE);
    lcdFillCircle(x + 20, y , 9, COLOR_WHITE);


}
void CloudThunder (int x, int y){
    // Draw the cloud body using rounded rectangles and circles
	lcdFillRoundRect(x, y, 30, 10, 5, COLOR_WHITE);
    lcdFillCircle(x + 24, y + 2, 7, COLOR_WHITE);
    lcdFillCircle(x + 15, y , 9, COLOR_WHITE);

    // Draw the first thunderbolt using lines for a zigzag shape
        lcdDrawLine(x + 7, y + 10, x + 5, y + 15, COLOR_YELLOW);   // First thunderbolt
        lcdDrawLine(x + 5, y + 15, x + 9, y + 15, COLOR_YELLOW);
        lcdDrawLine(x + 9, y + 15, x + 6, y + 21, COLOR_YELLOW);

        // Draw the second thunderbolt (centered below the cloud)
        lcdDrawLine(x + 15, y + 10, x + 13, y + 15, COLOR_YELLOW);  // Second thunderbolt
        lcdDrawLine(x + 13, y + 15, x + 17, y + 15, COLOR_YELLOW);
        lcdDrawLine(x + 17, y + 15, x + 14, y + 21, COLOR_YELLOW);

        // Draw the third thunderbolt (right side)
        lcdDrawLine(x + 23, y + 10, x + 21, y + 15, COLOR_YELLOW);  // Third thunderbolt
        lcdDrawLine(x + 21, y + 15, x + 25, y + 15, COLOR_YELLOW);
        lcdDrawLine(x + 25, y + 15, x + 22, y + 21, COLOR_YELLOW);

        // Draw the first thunderbolt using lines for a zigzag shape
		lcdDrawLine(x + 7 + 1, y + 10, x + 5 + 1, y + 15, COLOR_YELLOW);   // First thunderbolt
		lcdDrawLine(x + 5 + 1, y + 15, x + 9 + 1, y + 15, COLOR_YELLOW);
		lcdDrawLine(x + 9 + 1, y + 15, x + 6 + 1, y + 21, COLOR_YELLOW);

		// Draw the second thunderbolt (centered below the cloud)
		lcdDrawLine(x + 15 + 1, y + 10, x + 13 + 1, y + 15, COLOR_YELLOW);  // Second thunderbolt
		lcdDrawLine(x + 13 + 1, y + 15, x + 17 + 1, y + 15, COLOR_YELLOW);
		lcdDrawLine(x + 17 + 1, y + 15, x + 14 + 1, y + 21, COLOR_YELLOW);

		// Draw the third thunderbolt (right side)
		lcdDrawLine(x + 23 + 1, y + 10, x + 21 + 1, y + 15, COLOR_YELLOW);  // Third thunderbolt
		lcdDrawLine(x + 21 + 1, y + 15, x + 25 + 1, y + 15, COLOR_YELLOW);
		lcdDrawLine(x + 25 + 1, y + 15, x + 22 + 1, y + 21, COLOR_YELLOW);
}

uint16_t interpolateColor(uint16_t color1, uint16_t color2, float ratio) {
    uint8_t r1 = (color1 >> 11) & 0x1F;
    uint8_t g1 = (color1 >> 5) & 0x3F;
    uint8_t b1 = color1 & 0x1F;

    uint8_t r2 = (color2 >> 11) & 0x1F;
    uint8_t g2 = (color2 >> 5) & 0x3F;
    uint8_t b2 = color2 & 0x1F;

    uint8_t r = r1 + (r2 - r1) * ratio;
    uint8_t g = g1 + (g2 - g1) * ratio;
    uint8_t b = b1 + (b2 - b1) * ratio;

    return (r << 11) | (g << 5) | b;
}

void DrawGradientLine(int16_t x, int16_t y, int16_t length, int16_t thickness, uint16_t colorStart, uint16_t colorEnd , int Vitri) {
    // Radius for rounded ends, equal to half the thickness of the line
    int16_t radius = thickness / 2;

    // Draw the main gradient line with thickness
    for (int16_t i = 0; i < length; i++) {
        float ratio = (float)i / length;  // Calculate the interpolation ratio
        uint16_t color = interpolateColor(colorStart, colorEnd, ratio);

        // Draw a vertical line of "thickness" pixels at each position along the gradient line
        for (int16_t j = -radius + 1; j <= radius; j++) {
            lcdDrawPixel(x + i , y + j, color);  // Center the thickness vertically
        }
    }

    // Draw rounded ends as circles with colors matching the ends of the gradient
    lcdFillCircle(x, y  , radius - 1, colorStart);                 // Start of the line
    lcdFillCircle(x + length - 1, y , radius - 1, colorEnd);      // End of the line
    lcdDrawCircle(x + Vitri, y , radius , COLOR_THEME_SKYBLUE_BASE);
    lcdDrawCircle(x + Vitri, y , radius - 1 , COLOR_WHITE);
}

void WeatherDay (int x , int y , int TempMax , int TempMin , int weather_code , int current_temp ){
	int rate = weather_code;

	  lcdSetCursor(x + 5, y + 20);
	  lcdSetTextFont(&Font16);
	  lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);

	  if (weather_code <= 57){
	  		  CloudSun(x + 45, y + 14);
	  	  }
	  	  else if (weather_code <= 67) {
	  		  //Rain
	  		  CloudRain(x + 45 , y + 10 );
	  	  }
	  	  else if (weather_code < 95){
	  		  if (weather_code ==80 || weather_code == 81 || weather_code == 82){
	  			  //Rain
	  			  CloudRain(x + 45 , y + 10);
	  		  }
	  		  else{
	  			  //Snow
	  		  }
	  	  }
	  	  else{
	  		  //Thunder
	  		  CloudThunder (x + 45, y + 9);
	  	  }
	  lcdSetTextFont(&Font12);
	  if (rate >= 10)
	  {
		  lcdSetCursor(x + 53, y + 32);
		  lcdPrintf("%d", rate);

	  }
	  else{
		  lcdSetCursor(x + 57, y + 32);
		  lcdPrintf("%d", rate);
	  }

	  lcdSetCursor(x + 5, y + 20);
	  lcdSetTextFont(&Font16);
	  lcdPrintf("Now");
	  lcdSetCursor(x + 93, y + 20);
	  if (10 <= TempMin)
	  {
		lcdSetCursor(x + 93, y + 20);
		lcdPrintf("%d" , TempMin);
		lcdDrawCircle( x + 121 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 121 , y + 19 , 2, COLOR_WHITE);
	  }
	  else if ( 0 <  TempMin)
	  {
		lcdSetCursor(x + 104, y + 20);
		lcdPrintf("%d" , TempMin);
		lcdDrawCircle( x + 121 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 121 , y + 19 , 2, COLOR_WHITE);
	  }
	  else if (-10 < TempMin )
	  {
		lcdSetCursor(x + 93, y + 20);
		lcdPrintf("%d" , TempMin);
		lcdDrawCircle( x + 121 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 121 , y + 19 , 2, COLOR_WHITE);
	  }
	  else
	  {
		lcdSetCursor(x + 82, y + 20);
		lcdPrintf("%d" , TempMin);
		lcdDrawCircle( x + 121 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 121 , y + 19 , 2, COLOR_WHITE);
	  }
	  double Dolon = TempMax - TempMin;
	  double Dolon1 = current_temp - TempMin;
	  int Vitri = (Dolon1 / Dolon) * 50;

	  DrawGradientLine(x + 133, y + 25 , 50 , 6 ,  COLOR_ORANGE , COLOR_RED , Vitri);

	  lcdSetCursor(x + 188, y + 20);
	  if (10 < TempMax)
	  {
		lcdSetCursor(x + 191, y + 20);
		lcdPrintf("%d" , TempMax);
		lcdDrawCircle( x + 218 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 218 , y + 19 , 2, COLOR_WHITE);
	  }
	  else if ( 0 <  TempMax)
	  {
		lcdSetCursor(x + 191, y + 20);
		lcdPrintf("%d" , TempMax);
		lcdDrawCircle( x + 207 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 207 , y + 19 , 2, COLOR_WHITE);
	  }
	  else if (-10 < TempMax )
	  {
		lcdSetCursor(x + 191, y + 20);
		lcdPrintf("%d" , TempMax);
		lcdDrawCircle( x + 218 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 218 , y + 19 , 2, COLOR_WHITE);
	  }
	  else
	  {
		lcdSetCursor(x + 191, y + 20);
		lcdPrintf("%d" , TempMax);
		lcdDrawCircle( x + 229 , y + 19 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 229 , y + 19 , 2, COLOR_WHITE);
	  }
}
void DrawGauge(int16_t centerX, int16_t centerY, int16_t radius) {
    // Vẽ cung tròn các đoạn màu
    lcdDrawThickArc(centerX, centerY, radius, 120, 180, 10, COLOR_GREEN);  // Đoạn xanh
    lcdDrawThickArc(centerX, centerY, radius, 180, 240, 10, COLOR_ORANGE); // Đoạn vàng
    lcdDrawThickArc(centerX, centerY, radius, 240, 300, 10, COLOR_RED);    // Đoạn đỏ

    // Vẽ kim chỉ báo
    int16_t needleAngle = 210; // Góc kim (giữa đoạn vàng)
    int16_t needleLength = radius - 10;
    int16_t x1 = centerX + (needleLength * cosf(needleAngle * M_PI / 180.0));
    int16_t y1 = centerY - (needleLength * sinf(needleAngle * M_PI / 180.0));
    lcdDrawLine(centerX, centerY, x1, y1, COLOR_WHITE);

    // Vẽ vòng tròn trung tâm
    lcdDrawCircle(centerX, centerY, 5, COLOR_WHITE);
}

void DrawThermometer(int x, int y, int height, int width, int fillHeight, uint16_t borderColor, uint16_t fillColor) {
    int bulbRadius = width / 2;                  // Radius of the bulb at the bottom
    int tubeWidth = width / 2;                   // Width of the thermometer tube
    int tubeHeight = height - bulbRadius * 2;    // Height of the thermometer tube

    // Draw the bulb at the bottom
    lcdFillCircle(x, y + height - bulbRadius, bulbRadius , borderColor);           // Outer circle for bulb
    lcdFillCircle(x, y + height - bulbRadius, bulbRadius - 2, COLOR_WHITE);       // Inner circle for bulb border
    lcdFillCircle(x, y + height - bulbRadius, bulbRadius - 3, fillColor);         // Inner fill for "mercury" in bulb

    // Draw the thermometer tube
    lcdFillRoundRect(x - tubeWidth / 2, y, tubeWidth, tubeHeight + 1 , tubeWidth / 2, borderColor);       // Outer tube border
    lcdFillRoundRect(x - tubeWidth / 2 + 2, y + 2, tubeWidth - 4, tubeHeight - 4, tubeWidth / 2 - 2, COLOR_WHITE); // Inner tube border

    // Fill the thermometer tube based on the fill height
    int filledTubeHeight = fillHeight * tubeHeight / height;  // Scale fill height to tube height
    lcdFillRect(x - tubeWidth / 2 + 2, y + tubeHeight - filledTubeHeight + 2, tubeWidth - 5, filledTubeHeight, fillColor);  // Fill tube with "mercury"

    // Draw finer scale marks on the side of the thermometer
    int markCount = 8;  // Increased number of marks for more detail
    int markSpacing = tubeHeight / markCount; // Spacing between marks
    for (int i = 0; i <= markCount; i++) {
        int markY = y + tubeHeight - i * markSpacing;
        int markLength = (i % 2 == 0) ? 8 : 5;  // Longer marks every other line for a clearer scale
        lcdDrawLine(x + tubeWidth / 2 + 3, markY, x + tubeWidth / 2 + 3 + markLength, markY, borderColor);
    }
}

void DrawIconHumidyti1 (int x, int y)
{
	lcdDrawImage(x, y , &bmicon1);
}

void DrawIconDate (int x, int y)
{
	lcdDrawImage(x, y , &bmicon2);
}
void DrawIconWeather (int x, int y)
{
	lcdDrawImage(x, y , &bmwether);
}
void DrawIconTem (int x, int y)
{
	lcdDrawImage(x, y , &bmTem);
}
void DrawIconWind (int x, int y)
{
	lcdDrawImage(x, y , &bmwind);
}
void TextTemperature16 (int x , int y , int temperature){
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdSetTextFont(&Font24);
	if (10 <= temperature)
	{
		lcdSetCursor(x + 5 , y );
		lcdPrintf("%d C\r\n" , temperature);
		lcdDrawCircle( x + 47 , y + 3 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 47 , y + 3 , 2, COLOR_WHITE);
	}
	else if ( 0 <  temperature)
	{
		lcdSetCursor(x + 12  , y );
		lcdPrintf("%d C\r\n" , temperature);
		lcdDrawCircle( x + 36 , y + 3 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 36 , y + 3 , 2, COLOR_WHITE);
	}
	else if (-10 < temperature )
	{
		lcdSetCursor(x + 5  , y );
		lcdPrintf("%d C\r\n" , temperature);
		lcdDrawCircle( x + 47 , y + 3 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 47 , y + 3 , 2, COLOR_WHITE);
	}
	else
	{
		lcdSetCursor(x - 3  , y );
		lcdPrintf("%d C\r\n" , temperature);
		lcdDrawCircle( x + 56 , y + 3 , 3, COLOR_WHITE);
		lcdDrawCircle( x + 56 , y + 3 , 2, COLOR_WHITE);
	}
}

void TextHumidyti16 (int x , int y , int humidyti){
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdSetTextFont(&Font24);
	if (humidyti < 10){
		lcdSetCursor(x  , y );
		lcdPrintf("%d/\r\n" , humidyti);
		lcdDrawCircle( x + 20 , y + 3 , 2, COLOR_WHITE);
		lcdDrawCircle( x + 29 , y + 16 , 2, COLOR_WHITE);
	}
	else {
		lcdSetCursor(x - 5  , y );
		lcdPrintf("%d/\r\n" , humidyti);
		lcdDrawCircle( x + 33 , y + 3 , 2, COLOR_WHITE);
		lcdDrawCircle( x + 41 , y + 15 , 2, COLOR_WHITE);
	}
}
void TextTime(int x , int y , char * current_time){
	lcdSetCursor(x, y);
	lcdSetTextFont(&Font12);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdPrintf("Time: %s" , current_time);
}
void TextDate(int x , int y , char * current_date){
	lcdSetCursor(x, y);
	lcdSetTextFont(&Font12);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdPrintf("Date: %s" , current_date);
}
void TextLocation(int x , int y , int choice)
{
	lcdSetCursor(x, y);
	lcdSetTextFont(&Font12);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
    if (choice == 1){
    	lcdPrintf("City: Ho Chi Minh");
    }
    else if(choice == 2){
    	lcdPrintf("City: Ha Noi");
    }
    else if(choice == 3){
    	lcdPrintf("City: Hai Phong");
	}
    else if(choice == 4){
    	lcdPrintf("City: Can Tho");
	}
    else if(choice == 5){
    	lcdPrintf("City: Da Nang");
	}
}
void TextSensor(int x , int y ,float temperature ,float humidity ){
	lcdSetCursor(x + 40, y);
	lcdSetTextFont(&Font16);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdPrintf("Sensor");
	lcdSetCursor(x + 12, y + 55);
	lcdPrintf("%d C", (int)temperature);
	if (temperature < 0 || temperature >= 10){
		lcdDrawCircle(x + 39, y + 56, 2, COLOR_WHITE);
		lcdDrawCircle(x + 39, y + 56, 3, COLOR_WHITE);
	}
	else {
		lcdDrawCircle(x + 28, y + 56, 2, COLOR_WHITE);
		lcdDrawCircle(x + 28, y + 56, 3, COLOR_WHITE);
	}
	lcdSetCursor(x + 90, y + 55);
	lcdPrintf("%d%%", (int)humidity);
}
void TextCloudRate(int x , int y , int cloud_rate ){
	lcdSetCursor(x + 10, y);
	lcdSetTextFont(&Font16);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdPrintf("Cloud");
	if (cloud_rate < 100){
		lcdSetCursor(x + 22, y + 55);
	}
	else {
		lcdSetCursor(x + 15, y + 55);
	}
	lcdPrintf("%d%%", cloud_rate);
}
void TextUV(int x , int y , float UV ){
	lcdSetCursor(x , y);
	lcdSetTextFont(&Font16);
	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdPrintf("UV: %.1f" , UV);
}
void DrawIconHot (int x, int y){
	  lcdDrawImage(x, y, &bmhot);
}
void DrawIconIce (int x, int y){
	  lcdDrawImage(x, y, &bmice);
}
void DrawIconTree (int x, int y){
	  lcdDrawImage(x, y, &bmtree1);
}
void DrawIconWater (int x, int y){
	  lcdDrawImage(x, y, &bmwater);
}
void DrawIconClockTem(int x , int y){
	lcdDrawImage(x, y, &bmclocktem);
}
void DrawIconClockHumi(int x , int y){
	lcdDrawImage(x, y, &bmclockhumi);
}
void DrawIconCloudRate(int x , int y){
	lcdDrawImage(x, y, &bmcloudrate);
}
void DrawIconLocation(int x , int y){
	lcdDrawImage(x, y, &bmlocation);
}
void DrawIconNext(int x , int y){
	lcdDrawImage(x, y, &bmnext);
}
void DrawIconNext1(int x , int y){
	lcdDrawImage(x, y, &bmnext1);
}
void DrawIconBack(int x , int y){
	lcdDrawImage(x, y, &bmBack);
}
void DrawIconOk(int x , int y){
	lcdDrawImage(x, y, &bmOK);
}
void DrawBackGround(int x , int y){
	lcdDrawImage(x, y, &bmCity);
}
void DrawConfirm(int x , int y){
	lcdDrawImage(x, y, &bmconfirm);
}
void DrawNameIcon(int x , int y){
	lcdDrawImage(x, y, &bmabc);
}
void OneDay(int x , int y, int MinTem , int MaxTem , int wind , char day_name[] , int day_code , char date[]){
	  lcdSetCursor(x + 7 , y - 6);
	  lcdSetTextFont(&Font12);
	  lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	  lcdPrintf(day_name);
	  lcdSetCursor(x , y + 7);
	  lcdPrintf(date);
	  lcdSetTextFont(&Font16);

	  lcdSetCursor(x + 103, y);
	  if (MinTem < 10){
		  if (MaxTem < 10){
			lcdPrintf("0%d-0%d", MinTem , MaxTem);
		  }
		  else {
			  lcdPrintf("0%d-%d", MinTem , MaxTem);
		  }
	  }
	  else{
		  if (MaxTem < 10){
			  lcdPrintf("%d-0%d", MinTem , MaxTem);
		  }
		  else {
			  lcdPrintf("%d-%d", MinTem , MaxTem);
		  }
	  }

	  lcdDrawCircle(x + 163, y - 2, 2, COLOR_WHITE);
	  lcdDrawCircle(x + 163, y - 2, 3, COLOR_WHITE);

	  if (day_code <= 57){
		  CloudSun(x + 51, y + 5);
	  }
	  else if (day_code <= 67) {
		  //Rain
		  CloudRain(x + 51 , y );
	  }
	  else if (day_code < 95){
		  if (day_code ==80 || day_code == 81 || day_code == 82){
			  //Rain
			  CloudRain(x + 51 , y );
		  }
		  else{
			  //Snow
		  }
	  }
	  else{
		  //Thunder
		  CloudThunder (x + 51, y - 1);
	  }

	  if (wind >= 10){
		  lcdSetCursor(x + 178, y);
		  lcdPrintf("%d",wind);

		  lcdSetCursor(x + 202, y+1);
		  lcdSetTextFont(&Font12);
		  lcdPrintf("km/h");
//		  lcdDrawCircle(x + 213, y + 2, 2, COLOR_WHITE);
//		  lcdDrawCircle(x + 218, y + 10, 2, COLOR_WHITE);
	  }
	  else {
		  lcdSetCursor(x + 178, y);
		  lcdPrintf("0%d",wind);
		  lcdSetCursor(x + 202, y+1);
		  lcdSetTextFont(&Font12);
		  lcdPrintf("km/h");
//		  lcdDrawCircle(x + 209, y + 2, 2, COLOR_WHITE);
//		  lcdDrawCircle(x + 215, y + 10, 2, COLOR_WHITE);
	  }
}

void Screen0(){
	lcdFillRGB(COLOR_THEME_SKYBLUE_BASE);
	lcdDrawImage(60, 100, &bmLoading);
}
void Screen1(int TempMax , int TempMin , int current_temp ,int current_humi , int current_code , int current_cloud , char * current_time , char * current_date , int choice){
	lcdFillRGB(COLOR_THEME_SKYBLUE_BASE);
	// Divide layout
	lcdDrawRoundRect(158, 25, 40, 40, 5, COLOR_THEME_SKYBLUE_SHADOW);
	DrawIconLocation(161, 28);

	lcdDrawRoundRect(199, 25, 40, 40, 5, COLOR_THEME_SKYBLUE_SHADOW);
	DrawIconNext1(202, 28);

	lcdDrawRoundRect(76, 70, 163, 110, 6, COLOR_THEME_SKYBLUE_SHADOW);

	lcdDrawRoundRect(1, 70, 73, 110, 6, COLOR_THEME_SKYBLUE_SHADOW);

	lcdDrawRoundRect(1, 187, 238, 50, 6, COLOR_THEME_SKYBLUE_SHADOW);


	lcdDrawRoundRect(1, 240, 150, 78, 6, COLOR_THEME_SKYBLUE_SHADOW);
	lcdDrawLine(1 , 262 , 150 , 262 , COLOR_BLACK);

	lcdDrawLine(76, 262, 76, 317, COLOR_BLACK);

	lcdDrawRoundRect(155, 240, 84, 78, 6, COLOR_THEME_SKYBLUE_SHADOW);
	lcdDrawLine(155 , 262 , 239 , 262 , COLOR_BLACK);


	// Cac text va icon ban dau
	TextTitle( 22 , 2);
	TextTime(5, 25 , current_time);
	TextDate(5, 40 , current_date);
	TextLocation(5, 55 , choice);
	DrawCloud (3 , 80);
	TextUV(5 , 160 , 1.2);

	// Ve icon nhiet do va hien thi nhiet do
	DrawThermometer(95, 80 , 40, 12, 25, COLOR_BLACK, COLOR_RED);
	TextTemperature16(120, 95, current_temp);

	if (current_temp >= 29){
	  DrawIconHot(205 , 85);
	}
	else if ( current_temp < 20){
	  DrawIconIce(205 , 85);
	}
	else {
	  DrawIconTree(205 , 85);
	}


	// Ve icon do am va hien thi do am
	DrawIconWater(205 , 138);
	TextHumidyti16 (140, 146 , current_humi);
	DrawIconHumidyti1(77, 130);

	// Hien thi nhiet do max min trong ngay
	WeatherDay(0, 190 , TempMax , TempMin , current_code , current_temp);

	// Hien thi sensor
	TextSensor(5, 245 , current_temp , current_humi);
	TextCloudRate(159, 245, current_cloud);
	DrawIconClockTem(14 , 265);
	DrawIconClockHumi(88, 265);
	DrawIconCloudRate(170, 265);
}

void Screen2(int Max_temp[] , int Min_temp[] , int day_code[] , char day_name[][4] , int Wind_speed[] , char date [][6] , char * current_time , char * current_date , int choice){

	lcdFillRGB(COLOR_THEME_SKYBLUE_BASE);

	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdSetTextFont(&Font20);
	lcdSetCursor(28, 6);
	lcdPrintf("Forecast Days");

	//
	lcdDrawRoundRect(199, 23, 40, 40, 5, COLOR_THEME_SKYBLUE_SHADOW);
	DrawIconBack(202, 26);
	TextTime(5, 26 , current_time);
	TextDate(5, 38 , current_date);
	TextLocation(5, 50 , choice);


	  DrawIconDate(5, 65);
	  DrawIconWeather(55, 65);
	  DrawIconTem(120, 65);
	  DrawIconWind(195, 72);

	  lcdDrawLine(0, 65, 0, 319, COLOR_BLACK);
	  lcdDrawLine(239, 65, 239, 319, COLOR_BLACK);
	  lcdDrawLine(50, 65, 50, 319, COLOR_BLACK);
	  lcdDrawLine(100, 65, 100, 319, COLOR_BLACK);
	  lcdDrawLine(180, 65, 180, 319, COLOR_BLACK);

	  lcdDrawLine(0, 65 , 239, 65  , COLOR_BLACK);
	  lcdDrawLine(0, 108, 239, 108 , COLOR_BLACK);
	  lcdDrawLine(0, 143, 239, 143 , COLOR_BLACK);
	  lcdDrawLine(0, 178, 239, 178 , COLOR_BLACK);
	  lcdDrawLine(0, 213, 239, 213 , COLOR_BLACK);
	  lcdDrawLine(0, 248, 239, 248 , COLOR_BLACK);
	  lcdDrawLine(0, 283, 239, 283 , COLOR_BLACK);
	  for (int i = 1 ; i < 7 ; i++)
	  {

		  OneDay(7, 85 + i * 35, Min_temp[i]  , Max_temp[i] , Wind_speed[i] , day_name[i] , day_code[i] , date[i]);
	  }
}
void Screen3(int choice)
{
	DrawBackGround(0, 0);
	lcdSetTextFont(&Font24);
	lcdFillRoundRect(35,6, 170, 27, 13  , COLOR_THEME_SKYBLUE_BASE);
	lcdDrawRoundRect(35,6, 170, 28, 13  , COLOR_BLACK);
	lcdSetCursor(52, 9);

	lcdSetTextColor(COLOR_WHITE, COLOR_THEME_SKYBLUE_BASE);
	lcdPrintf("Location");


	lcdFillRoundRect(20,50, 200, 35, 6, COLOR_LIGHTGREY);
	lcdFillRoundRect(20,95, 200, 35, 6, COLOR_LIGHTGREY);
	lcdFillRoundRect(20,140, 200, 35, 6, COLOR_LIGHTGREY);
	lcdFillRoundRect(20,185, 200, 35, 6, COLOR_LIGHTGREY);
	lcdFillRoundRect(20,230, 200, 35, 6, COLOR_LIGHTGREY);

	lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
	lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
	lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
	lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
	lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);

	lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
	lcdSetTextFont(&Font20);
	lcdSetCursor(25, 60);
	lcdPrintf("TP.HCM");



	lcdSetTextFont(&Font20);
	lcdSetCursor(25, 105);
	lcdPrintf("Ha Noi");


	lcdSetTextFont(&Font20);
	lcdSetCursor(25, 150);
	lcdPrintf("Hai Phong");


	lcdSetTextFont(&Font20);
	lcdSetCursor(25, 195);
	lcdPrintf("Can Tho");

	lcdSetTextFont(&Font20);
	lcdSetCursor(25, 240);
	lcdPrintf("Da Nang");

	if(choice == 1){
		lcdFillRoundRect(20,50, 200, 35, 6, COLOR_GREEN);
		lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
		lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
		lcdSetTextFont(&Font20);
		lcdSetCursor(25, 60);
		lcdPrintf("TP.HCM");

	}
	else if(choice == 2){
		lcdFillRoundRect(20,95, 200, 35, 6, COLOR_GREEN);
		lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
		lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
		lcdSetTextFont(&Font20);
		lcdSetCursor(25, 105);
		lcdPrintf("Ha Noi");
	}
	else if(choice == 3){
		lcdFillRoundRect(20,140, 200, 35, 6, COLOR_GREEN);
		lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
		lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
		lcdSetTextFont(&Font20);
		lcdSetCursor(25, 150);
		lcdPrintf("Hai Phong");
	}
	else if(choice == 4){
		lcdFillRoundRect(20,185, 200, 35, 6, COLOR_GREEN);
		lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
		lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
		lcdSetTextFont(&Font20);
		lcdSetCursor(25, 195);
		lcdPrintf("Can Tho");
	}
	else if(choice == 5){
		lcdFillRoundRect(20,230, 200, 35, 6, COLOR_GREEN);
		lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);
		lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
		lcdSetTextFont(&Font20);
		lcdSetCursor(25, 240);
		lcdPrintf("Da Nang");
	}

	//
	lcdFillRoundRect(100, 275, 40, 39, 5, COLOR_THEME_SKYBLUE_BASE);
	lcdDrawRoundRect(100, 275, 40, 40, 5, COLOR_THEME_SKYBLUE_SHADOW);
	DrawIconOk(103, 278);
}


