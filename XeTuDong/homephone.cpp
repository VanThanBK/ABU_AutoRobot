/*
This is arduino graphic library Version 1.0, that is designed only for homephone LCD (Viettel phone 10 pins)

This library provide a commond way to draw simple basic things (draw pixel, line, rectangle, circles, triangle...etc).
Some functions is refer from Adafruit Industries.

you can change different pin to hookup Arduino with the LCD as this library is only used software SPI connection.

============
Copyright (c) 2013 Adafruit Industries.  All rights reserved.
============
if you want to modify, make sure that include the information above to respect author. and please contribute it.

The detail documentation of using this library is also public on this address: pls feel free to download it.

WE DON'T HAVE ANY RESPONSIBILITY IN CASE THAT YOUR DEVICE RUN INTO SOME ISSUE AFTER USE THIS SOFTWARE.
BUT NORMALLY WE TESTED AND IT'S COMPATIBLE WITH HOMEPHONE (10 PINS).

if there is any thing you want to ask please contact me:
Contact name: TRAN NHAT THANH
Email address: NHATTHANH228@GMAIL.COM
i'd really appreciate if you send us any feedback.
============
Edit for TIVA C Launchpad by HocARM.org
*/

//#include "Arduino.h"
//#include "pins_arduino.h"
#include <Energia.h>
#include <stdlib.h>
#include "homephone.h"
#include "glcdfont.c"
// #include <avr/pgmspace.h>

#define allowUpdatePartially
//#define debug


const uint8_t pagemap[] {0, 1, 2, 3, 4, 5, 6, 7};
uint8_t lcdBuffer[1024] = {
};


#ifdef allowUpdatePartially
static uint8_t xUpdateMin, yUpdateMin, xUpdateMax, yUpdateMax;
#endif

//this will help to update screen parrtially. much faster that update whole screen each time data changed.
static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax) {
#ifdef allowUpdatePartially
	if (xmin < xUpdateMin) xUpdateMin = xmin;
	if (xmax > xUpdateMax) xUpdateMax = xmax;
	if (ymin < yUpdateMin) yUpdateMin = ymin;
	if (ymax > yUpdateMax) yUpdateMax = ymax;
#endif
}

void homephone::lcdWrite(uint8_t command, byte c) {

	digitalWrite(cs, LOW);
	if (command) {
		digitalWrite (a, HIGH);

	}
	else {
		digitalWrite (a, LOW);
	}
	shiftOut (sdin, slk, MSBFIRST, c);
}

void homephone::begin(void) {
	pinMode(sdin, OUTPUT);
	pinMode(slk, OUTPUT);
	pinMode(a, OUTPUT);
	pinMode(rst, OUTPUT);
	pinMode(cs, OUTPUT);

	digitalWrite (rst, LOW);
	delay(515);
	digitalWrite(rst, HIGH);
	lcdWrite (0, SET_BIAS_9);
	lcdWrite (0, SOFTWARE_RESET);
	lcdWrite (0, DISPLAY_ON);
	lcdWrite (0, POWER_CONTROL );
	lcdWrite(0, 0x26); //regulation ratio
	lcdWrite(0, 0x2F);
	lcdWrite(0, 0xA4);

	//for debuging
#ifdef debug
	Serial.begin(9600);
#endif
}

void homephone::setContrast (uint8_t c) {
	lcdWrite(0, 0x81);
	lcdWrite(0, 0 | (c & 0x3f));
}

void homephone::display() {
	uint8_t col, maxcol, i;
	//check if page is apart of update
	for (i  = 0; i < 8; i++) {
#ifdef allowUpdatePartially
		if (yUpdateMin >= ((i + 1) * 8)) {
			continue;  //if no then skip it!
		}
		if (yUpdateMax < i * 8) {
			break;
		}
#endif

		//set page
		lcdWrite(0, SET_PAGE | pagemap[i]);

#ifdef allowUpdatePartially
		col = xUpdateMin; 			//update from begining
		maxcol = xUpdateMax;		//to the maximum col is going to used
#else
		col = 0 ;					//update from begining
		maxcol = LCDWIDTH - 1; 		//to the end of col (0 - 127)
#endif

		//set col
		lcdWrite(0, COL_LSB | ((col) & 0xf));
		lcdWrite(0, COL_MSB | ((col) >> 4) & 0xf);


		for (; col <= maxcol; col++) {
			lcdWrite(1, lcdBuffer[(128 * i) + col]); //transfer data from buffer to lcd
		}

	}
#ifdef allowUpdatePartially
	xUpdateMin = LCDWIDTH - 1;
	xUpdateMax = 0;
	yUpdateMax = 0;
	yUpdateMin = LCDHEIGHT - 1;
#endif
}

void homephone::clear() {
	updateBoundingBox (0, 0, 127, 63);
	for (int i = 0; i < 1024; i++) {
		lcdBuffer[i] = 0;
	}
	display();
}

void homephone::drawPixel(uint8_t x, uint8_t y, uint8_t color) {
	if ((x >= LCDWIDTH) || (y >= LCDHEIGHT) || (x < 0) || (y < 0)) {

		return;
	}
	if (color) {
		lcdBuffer[x + (y / 8) * LCDWIDTH] |= 1 << (y % 8);
	}
	else {
		lcdBuffer[x + (y / 8) * LCDWIDTH] &= ~(1 << (y % 8));
	}
	updateBoundingBox(x, y, x, y);
}
//Ham hiển thị tất cả các điểm trên màn hình
void homephone::AllPixel(uint8_t val) {
	if (val) {
		lcdWrite(0, CMD_ALL_PIXEL);
	}
	else
	{
		lcdWrite(0, CMD_NONE_ALL_PIXEL);
	}

}
bool homephone::GetPixel(int16_t x, int16_t y) {
	bool i;
	if ((x >= LCDWIDTH) || (y >= LCDHEIGHT) || (x < 0) || (y < 0))
	{
		return 0;
	}
	// i = (lcdBuffer[x + (y / 8) * LCDWIDTH] >> (1 << (y % 8))) & 0x1;
	i = (lcdBuffer[x + (y / 8) * LCDWIDTH] >> (7 - (y % 8))) & 0x1;
	return i; //QUÉT BỘ NHỚ ĐỆM
}
void homephone::drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {

	//check slope line.
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			drawPixel(y0, x0, color);
		} else {
			drawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
	updateBoundingBox (x0, y0, x1, y1);
}
void homephone::drawHorizontalLine( long x1, long x2, long y0, bool color) {
	if (x1 > x2) {
		swap(x1, x2);
	}

	int16_t hieu = x2 - x1;
	for ( int16_t x = x1; x <= x2; x++) {
		drawPixel(x, y0, color);
	}
	updateBoundingBox(x1, y0, x1 + hieu, y0);
}

void homephone::drawVerticalLine(long x0, long y1, long y2, bool color) {
	if (y1 > y2) {
		swap(y1, y2);
	}
	int16_t hieu = y2 - y1;
	for ( int16_t y = y1; y <= y2; y++) {
		drawPixel( x0, y, color);

	}
	updateBoundingBox(x0, y1, x0, y1 + hieu);
}

void homephone::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
	// drawLine(x, y, x + w, y, color) ; //drawLine1
	// drawLine(x, y, x, y + h, color) ; //drawLine2
	// drawLine(x, y + h, x + w, y + h, color); //drawLine 3
	// drawLine(x + w, y, x + w, y + h, color); //drawLine 4
	// updateBoundingBox(x, y, x + w, y + h); //update screen faster
	w += x;
	h += y;
	for (int16_t i = x; i < w; i++) {
		drawPixel(i, y, color);
		drawPixel(i, h - 1, color);
	}
	for (int16_t i = y; i < h; i++) {
		drawPixel(x, i, color);
		drawPixel(w - 1, i, color);
	}

	updateBoundingBox(x, y, w, y);// cạnh trên
	updateBoundingBox(x, h, w, h);// cạnh dưới
	updateBoundingBox( x, y, x, h); //cạnh trái
	updateBoundingBox(w, y, w, h); // cạnh phải
}

void homephone::fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
	// for (int i = y; i <= y + h; i++) {
	// 	drawLine(x, i , x + w, i, color);
	// }
	// updateBoundingBox (x, y, x + w, y + h);
	w += x;
	h += y;

	for (int16_t i = x; i < w; i++) {
		for (int16_t j = y; j < h; j++) {
			drawPixel(i, j, color);
		}
	}
	updateBoundingBox(x, y, w, h);

}

void homephone::drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
	if (r < 1) {
		return;
	}
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawPixel(x0  , y0 + r, color);
	drawPixel(x0  , y0 - r, color);
	drawPixel(x0 + r, y0  , color);
	drawPixel(x0 - r, y0  , color);


	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		int16_t x0x = x0 + x, x0y = x0 + y, x0_x = x0 - x, x0_y = x0 - y;
		int16_t y0x = y0 + x, y0y = y0 + y, y0_x = y0 - x, y0_y = y0 - y;

		drawPixel(x0x, y0_y, color);// 1/2 góc phần tư thứ 1-trên
		drawPixel(x0y, y0_x, color);//1/2 góc phần tư thứ 1-dưới
		drawPixel(x0_x, y0_y, color);// 1/2 góc phần tư thứ 2-trên
		drawPixel(x0_y, y0_x, color);// 1/2 góc phần tư thứ 2-dưới
		drawPixel(x0_y, y0x, color);// 1/2 góc phần tư thứ 3-trên
		drawPixel(x0_x, y0y, color);// 1/2 góc phần tư thứ 3-dưới
		drawPixel(x0y, y0x, color);// 1/2 góc phần tư thứ 4-trên
		drawPixel(x0x, y0y, color);// 1/2 góc phần tư thứ 4-dưới
		//drawPixel(x0 + x, y0 + y, color);
		//drawPixel(x0 - x, y0 + y, color);
		//drawPixel(x0 + x, y0 - y, color);
		//drawPixel(x0 - x, y0 - y, color);
		//drawPixel(x0 + y, y0 + x, color);
		//drawPixel(x0 - y, y0 + x, color);
		//drawPixel(x0 + y, y0 - x, color);
		//drawPixel(x0 - y, y0 - x, color);
	}
	updateBoundingBox (x0 - r, y0 - r, x0 + r, y0 + r);
}

void homephone::fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
	if (r < 1) {return;} //thoát
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	for (int16_t i = y0 - r; i <= y0 + r; i++) {
		drawPixel(x0, i, color);
	}

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		int16_t y0y = y0 + y;
		int16_t y0x = y0 + x;
		for (int16_t i = y0 - y; i <= y0y; i++) {
			drawPixel(x0 + x, i, color);
			drawPixel(x0 - x, i, color);
		}
		for (int16_t i = y0 - x; i <= y0x; i++) {
			drawPixel(x0 + y, i, color);
			drawPixel(x0 - y, i, color);
		}
	}

	updateBoundingBox(x0 - r, y0 - r, x0 + r, y0 + r);

}
void homephone::Plot4EllipsePoint16_ts(uint8_t cx, uint8_t  cy, uint8_t x, uint8_t y, uint8_t color, uint8_t fill)
{
	int16_t x1, x2, y0a, y0b;
	x1 = cx - x;
	x2 = cx + x;
	y0a = cy + y;
	y0b = cy - y;
	if (fill)
	{	// to fill rather than draw a line, plot between the point16_ts
		drawHorizontalLine(x1, x2, y0a, color);
		//drawline(CX+X, CY+Y, CX-X, CY+Y, color);
		drawHorizontalLine(x1, x2, y0b, color);
		//drawline(CX-X, CY-Y, CX+X, CY-Y, color);
	}
	else
	{
		drawPixel(x2, y0a, color); //{point16_t in quadrant 1}

		drawPixel(x1, y0a, color); //{point16_t in quadrant 2}

		drawPixel(x1, y0b, color); //{point16_t in quadrant 3}

		drawPixel(x2, y0b, color); //{point16_t in quadrant 4}

	}
}
void homephone::DrawEllipse_private(uint8_t cx, uint8_t  cy, uint8_t xradius, uint8_t yradius, uint8_t color, uint8_t fill)
{
// ported the algorithm by John Kennedy found at
// http://homepage.smc.edu/kennedy_john/belipse.pdf
//
	if ((xradius < 1) || (yradius < 1)) {
		return;//thoát
	}
	uint8_t x, y;
	uint8_t xchange, ychange;
	uint8_t EllipseError;
	uint8_t TwoASquare, TwoBSquare;
	uint8_t StoppingX, StoppingY;
	TwoASquare = 2 * xradius * xradius;
	TwoBSquare = 2 * yradius * yradius;
	x = xradius;
	y = 0;
	xchange = yradius * yradius * (1 - 2 * xradius);
	ychange = xradius * xradius;
	EllipseError = 0;
	StoppingX = TwoBSquare * xradius;
	StoppingY = 0;

	while ( StoppingX >= StoppingY )
	{
		Plot4EllipsePoint16_ts(cx, cy, x, y, color, fill);

		y++;
		StoppingY = StoppingY + TwoASquare;
		EllipseError = EllipseError + ychange;
		ychange = ychange + TwoASquare;
		if ((2 * EllipseError + xchange) > 0 ) {
			x--;
			StoppingX = StoppingX - TwoBSquare;
			EllipseError = EllipseError + xchange;
			xchange = xchange + TwoBSquare;
		}
	}

	y = yradius;
	x = 0;
	ychange = xradius * xradius * (1 - 2 * yradius);
	xchange = yradius * yradius;
	EllipseError = 0;
	StoppingY = TwoASquare * yradius;
	StoppingX = 0;
	while ( StoppingY >= StoppingX )
	{
		Plot4EllipsePoint16_ts(cx, cy, x, y, color, fill);
		x++;
		StoppingX = StoppingX + TwoBSquare;
		EllipseError = EllipseError + xchange;
		xchange = xchange + TwoBSquare;
		if ((2 * EllipseError + ychange) > 0 ) {
			y--;
			StoppingY = StoppingY - TwoASquare;
			EllipseError = EllipseError + ychange;
			ychange = ychange + TwoASquare;
		}
	}
}
void homephone::drawElip(uint8_t xcenter, uint8_t  ycenter, uint8_t   xradius,  uint8_t  yradius, uint8_t color)
{
	DrawEllipse_private(xcenter, ycenter, xradius, yradius, color, 0);
}

void homephone::fillElip(uint8_t xcenter, uint8_t  ycenter, uint8_t xradius, uint8_t yradius, uint8_t color)
{
	DrawEllipse_private(xcenter, ycenter, xradius, yradius, color, 1);
}

void homephone::drawTriangle (uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint8_t color) {
	drawLine(x3, y3, x1, y1, color);
	drawLine(x1, y1, x2, y2, color);
	drawLine(x2, y2, x3, y3, color);
	// updateBoundingBox (x1, y0, x2, y2);
}

void homephone::fillTriangle (uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint8_t color) {
	uint8_t sl, sx1, sx2;
	double m1, m2, m3;
	if (y2 > y3)
	{
		swap(x2, x3);
		swap(y2, y3);
	}
	if (y1 > y2)
	{
		swap(x1, x2);
		swap(y1, y2);
	}
	m1 = (double)(x1 - x2) / (y1 - y2);
	m2 = (double)(x2 - x3) / (y2 - y3);
	m3 = (double)(x3 - x1) / (y3 - y1);
	for (sl = y1; sl <= y2; sl++)
	{
		sx1 = m1 * (sl - y1) + x1;
		sx2 = m3 * (sl - y1) + x1;
		if (sx1 > sx2)
			swap(sx1, sx2);
		drawHorizontalLine(sx1, sx2, sl, color);
		//drawline(sx1, sl, sx2, sl, color);
	}
	for (sl = y2; sl <= y3; sl++)
	{
		sx1 = m2 * (sl - y3) + x3;
		sx2 = m3 * (sl - y1) + x1;
		if (sx1 > sx2)
			swap(sx1, sx2);
		drawHorizontalLine(sx1, sx2, sl, color);
		//drawline(sx1, sl, sx2, sl, color);
	}
}

void homephone::drawChar (uint8_t x, uint8_t y, unsigned char c, uint8_t color, uint8_t bg) {
	if ((x >= 128 ) || (y >= 64) || (x < 0) || (y < 0)) {
		return;
	}

	for (int8_t i = 0; i < 6; i++) {
		uint8_t line;
		if (i == 5) {
			line = 0x0;
		}
		else {
			line = pgm_read_byte (font + (c * 5) + i);
#ifdef debug
			Serial.print(line, HEX);
#endif
		}
		for (int8_t j = 0; j < 8; j++) {
			if (line & 0x1) {
				drawPixel (x + i, y + j, color);
			}
			else if (bg != color) {
				drawPixel (x + i, y + j, bg);
			}
			line >>= 1;
		}

	}
}

void homephone::drawCorner(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, int16_t color) {
// Thủ thuật
//vẽ 4 cạnh hcn + 4 góc phần tư của hình tròn

	if ( (r > h / 2) || (r > w / 2)) {
		return;// thoát
	}
	int16_t xr = x + r, xw = x + w, xw_r = x + w - r;
	int16_t yr = y + r, yh = y + h, yh_r = y + h - r;
	//cạnh trên
	drawHorizontalLine(xr, xw_r, y, color);
	// cạnh trái
	drawVerticalLine(x, yr, yh_r, color);
	//cạnh dưới
	drawHorizontalLine( xr, xw_r, yh, color);
	// cạnh phải
	drawVerticalLine(xw, yr, yh_r, color);
	if (r < 1) {

		return;//thoát
	}
	// Bắt đầu vẽ góc
	int16_t f = 1 - r;
	int16_t ddF_a = 1;
	int16_t ddF_b = -2 * r;
	int16_t a = 0;
	int16_t b = r;
	while (a < b) {
		if (f >= 0) {
			b--;
			ddF_b += 2;
			f += ddF_b;
		}
		a++;
		ddF_a += 2;
		f += ddF_a;

		int16_t xw_ra = xw_r + a, xw_rb = xw_r + b;
		int16_t xr_a = xr - a, xr_b = xr - b;
		int16_t yr_a = yr - a, yr_b = yr - b;
		int16_t yh_ra = yh_r + a, yh_rb = yh_r + b;
		// bo góc phải- trên
		drawPixel( xw_ra, yr_b, color);
		drawPixel(xw_rb, yr_a, color);
		// bo góc trái-trên
		drawPixel(xr_a, yr_b, color);
		drawPixel(xr_b, yr_a, color);
		// bo góc trái dưới
		drawPixel(xr_b, yh_ra, color);
		drawPixel(xr_a, yh_rb, color);
		// bo góc phải- dưới
		drawPixel(xw_rb, yh_ra, color);
		drawPixel(xw_ra, yh_rb, color);


	}

	updateBoundingBox(x, y, x + w, y + h); //update screen faster
}

void homephone::fillCorner(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, int16_t color) {
// đành lười biếng kiểu này
// vẽ 4 hình tròn
	int16_t xw_r = x + w - r, yr = y + r, xr = x + r, yh_r = y + h - r;
	int16_t w_2r = w - 2 * r, h_2r = h - 2 * r;
	fillCircle(xw_r, yr, r, color);
	fillCircle(xr, yr, r, color);
	fillCircle(xr, yh_r, r, color);
	fillCircle(xw_r, yh_r, r, color);
// vẽ hình chữ nhật chèn vào
	fillRect(xr, y, w_2r, h + 1, color);
	fillRect(x, yr, r, h_2r, color);
	fillRect(xw_r, yr, r + 1, h_2r, color);

	updateBoundingBox(x, y, x + w, y + h); //update screen faster
// xong
}
// void homephone::drawBitmap(uint8_t x, uint8_t y, const uint8_t * bitmap, uint8_t w, uint8_t h, uint8_t color, uint8_t bg) {
void homephone::drawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bitmap, uint8_t color) {
	uint8_t non = !color;

	for (uint8_t i = 0; i < w; i++ ) {

		for (uint8_t j = 0; j < h; j++) {


			if (bitRead( pgm_read_byte(bitmap + i + (j / 8)*w), j % 8)) { // font: địa chỉ của mảng font[] là tên mảng đó

				drawPixel(x + i, y + j, color);
			} else {

				drawPixel(x + i, y + j, non);
			}
		}
	}

	updateBoundingBox(x, y, x + w, y + h);
}
void homephone::Plus_Bitmap(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, const uint8_t *bitmap , uint8_t goc, uint8_t mirror, uint8_t color) {
// xoay ảnh bitmap trong4 góc 0-90-180-270
	uint8_t x, y;
	uint8_t notcolor = !color;
	uint8_t w_1, h_1;
	w_1 = w - 1;
	h_1 = h - 1;
	for (uint8_t i = 0; i < w; i++ ) { //con trỏ dich byte
		for (uint8_t j = 0; j < h; j++) { // con trỏ dịch bit
			switch (goc) {
			case 0:
				y = y0 + j; //0
				if (mirror) {

					x = (x0 + w_1) - i; // xoay 0 độ  gương
				} else {

					x = x0 + i; // xoay 0 độ  thường
				}
				break;
			case 90:
				y = (y0 + w_1) - i; //90
				if (mirror) {
					x = (x0 + h_1) - j; // xoay 90 độ  gương
				} else {
					x = x0 + j; // xoay 90 độ  thường
				}
				break;
			case 180:

				y = (y0 + h_1) - j; //180
				if (mirror) {
					x = x0 + i; // xoay 180 độ gương

				} else {

					x = (x0 + w_1) - i; // xoay 180 độ thường
				}
				break;
			case 270:
				y = y0 + i; //270
				if (mirror) {

					x = x0 + j; // xoay 270 độ gương

				} else {
					x = (x0 + h_1) - j; // xoay 270 độ thường

				}
				break;
			default:
				return;  //thoát
				break;

			}// switch

			if (pgm_read_byte(bitmap + i + (j / 8)*w) & (j % 8)) {
				drawPixel(x, y, color);
			} else {
				drawPixel(x, y, notcolor);
			}
		}
	}
	w += x0;
	h += y0;
	if ((goc == 0) || (goc == 90)) {
		updateBoundingBox(x0, y0, w, h);
	} else {

		updateBoundingBox(x0, y0, h, w);
	}
}
void homephone::setCursor(uint8_t x, uint8_t y) {
	cursor_x = x;
	cursor_y = y;
}

void homephone::setTextColor(uint8_t color, uint8_t bgColor) {
	textColor = color;
	textBgColor = bgColor;
}

size_t homephone::write(uint8_t c) {
	if ( c == '\n') {
		cursor_y += 8;
		cursor_x = 0;
	}
	else if (c == '\r') {

	}
	else {
		drawChar (cursor_x, cursor_y, c, textColor, textBgColor);
		cursor_x += 6;
	}
}

void  homephone::ascChar(int16_t x1, int16_t y1, unsigned char c, int16_t color) {
	int16_t non = !color;
	const unsigned char* dress = font + c * 5;
	for (byte i = 0; i < 5; i++ ) {
		for (byte j = 0; j < 8; j++ ) {

			if (bitRead( pgm_read_byte(dress + i), j)) { // font: địa chỉ của mảng font[] là tên mảng đó
				drawPixel(x1 + i, y1 + j, color);
			} else {
				drawPixel(x1 + i, y1 + j, non);
			}
		}
	}
	updateBoundingBox(x1, y1, x1 + 5, y1 + 8);
}
void homephone::ascString(int16_t x1, int16_t y1, const char *s , int16_t color) {
	int16_t x = x1;
	unsigned char c;
	while ((c = pgm_read_byte(s)) != 0) { // mối lần c++ thì địa chỉ của kí tự là c[0] do c=c[]
		// Mời bạn tìm hiểu thêm cách lưu biến tĩnh vào bộ nhớ flash trên trang arduino.vn
		ascChar(x, y1, c, color);
		//Serial.print16_tln(c[i]);
		s++;
		x += 6; // 6 pixels wide
		if (x + 6 >=  LCDWIDTH  ) {
			x = x1;    // ran out of this line
			y1 += 8;
		}
		if ( y1 > LCDHEIGHT)
			return;        // dừng vòng lặp
	}
}
void homephone::Asc_String_Zoom(int16_t x1, int16_t y1, const char *s , int16_t he_so_phong_to, int16_t color) {
	if (he_so_phong_to == 1) {
		ascString( x1,  y1, s , color) ;
		return;///thoát

	}

	int16_t x = x1;
	unsigned char c;
	while ((c = pgm_read_byte(s)) != 0) { // mối lần c++ thì địa chỉ của kí tự là c[0] do c=c[]
		// Mời bạn tìm hiểu thêm cách lưu biến tĩnh vào bộ nhớ flash trên trang arduino.vn
		Asc_Char_Zoom(x, y1, c, he_so_phong_to, color);
		//Serial.print16_tln(c[i]);
		s++;
		x += 6 * he_so_phong_to; // 6 pixels wide
		if (x + 6 * he_so_phong_to >=  LCDWIDTH  ) {
			x = x1;    // ran out of this line
			y1 += 8 * he_so_phong_to;
		}
		if ( y1 > LCDHEIGHT)
			return;        // dừng vòng lặp
	}

}
void  homephone::Asc_Char_Zoom(int16_t x1, int16_t y1, unsigned char c, int16_t he_so_phong_to, int16_t color) {
	ascChar( x1,  y1, c , color);	// vẽ

	//rồi phóng to
	if (he_so_phong_to > 1) {
		Zoom_In(x1, y1, 5, 7, he_so_phong_to, color ); // kích thước của font ascii trong thư viejn này là 5x7 cho mỗi kí tự
	}
}
void homephone::Zoom_In(int16_t x0, int16_t y0, uint16_t w0, uint16_t h0, uint8_t he_so_phong_to, int16_t color) {
	// hàm này có nhiệm vụ phóng to một vùng ảnh lên gấp 2, gấp 4, gấp 8 ..
// đầu

// b0:cấp phát một mảng để copy vùng ảnh
	uint16_t copy_buffer[w0][h0];
//bool *hh=new bool[h0];
	//b1: copy vùng ảnh tọa độ x0+w,y0+w kích thước 1xh0
	for ( uint16_t w = 0; w < w0; w++) {
		for ( uint16_t h = 0; h < h0; h++) {
			copy_buffer[w][h] = GetPixel( x0 + w, y0 + h); // kiểm tra xem điểm có được vẽ hay không

		}
	}

//b1.1: xóa vùng ảnh cũ
	for (int16_t x = x0; x < w0 + x0; x++) {
		for (int16_t y = y0; y < h0 + y0; y++) {
			drawPixel(x, y, white);
		}
	}
	//b2: vẽ lại vùng ảnh với tỷ lệ

	for ( uint16_t w = 0; w < w0; w++) {
		for ( uint16_t h = 0; h < h0; h++) {
			if (copy_buffer[w][h] == 1) {

				// phóng to điểm ảnh thành hình vuông kích thước he_so_phong_to*he_so_phong_to
				for (byte i = 0; i < he_so_phong_to; i++) {
					for (byte j = 0; j < he_so_phong_to; j++) {
						drawPixel( x0 + w * he_so_phong_to + i, y0 + h * he_so_phong_to + j, color);
					}
				}
			}
		}
	}
	updateBoundingBox(x0, y0, x0 + w0 * he_so_phong_to, y0 + h0 * he_so_phong_to);

}




















