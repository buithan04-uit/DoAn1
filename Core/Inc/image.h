/*
 * image.h
 *
 *  Created on: 31 ����. 2019 �.
 *      Author: tabur
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <stdint.h>

typedef struct _tImage
{
  uint16_t 					xSize;
  uint16_t 					ySize;
  uint16_t 					bytesPerLine;
  uint8_t					bitsPerPixel;
  const unsigned char* 		pData;
} sImage;

#define GUI_BITMAP			sImage
#define GUI_CONST_STORAGE	const

extern GUI_CONST_STORAGE GUI_BITMAP bmSTLogo;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname1;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname2;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname3;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname4;
extern GUI_CONST_STORAGE GUI_BITMAP bmCloud1;
extern GUI_CONST_STORAGE GUI_BITMAP bmCloud2;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname5;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname6;
extern GUI_CONST_STORAGE GUI_BITMAP bmyourname51;
extern GUI_CONST_STORAGE GUI_BITMAP bmBackGround;
extern GUI_CONST_STORAGE GUI_BITMAP bmSky1;
extern GUI_CONST_STORAGE GUI_BITMAP bmIcon;
extern GUI_CONST_STORAGE GUI_BITMAP bmBackGround2;
extern GUI_CONST_STORAGE GUI_BITMAP bmicon1;
extern GUI_CONST_STORAGE GUI_BITMAP bmicon2;
extern GUI_CONST_STORAGE GUI_BITMAP bmicon3;
extern GUI_CONST_STORAGE GUI_BITMAP bmice;
extern GUI_CONST_STORAGE GUI_BITMAP bmhot;
extern GUI_CONST_STORAGE GUI_BITMAP bmwind;
extern GUI_CONST_STORAGE GUI_BITMAP bmweather;
extern GUI_CONST_STORAGE GUI_BITMAP bmwater;
extern GUI_CONST_STORAGE GUI_BITMAP bmdoC;
extern GUI_CONST_STORAGE GUI_BITMAP bmhumi;
extern GUI_CONST_STORAGE GUI_BITMAP bmwether;
extern GUI_CONST_STORAGE GUI_BITMAP bmTem;
extern GUI_CONST_STORAGE GUI_BITMAP bmz6030061336714_5dbe1c34090d7e2346532e65515815be;
extern GUI_CONST_STORAGE GUI_BITMAP bmA;
extern GUI_CONST_STORAGE GUI_BITMAP bmtree;
extern GUI_CONST_STORAGE GUI_BITMAP bmtree1;
extern GUI_CONST_STORAGE GUI_BITMAP bmclocktem;
extern GUI_CONST_STORAGE GUI_BITMAP bmclockhumi;
extern GUI_CONST_STORAGE GUI_BITMAP bmnext;
extern GUI_CONST_STORAGE GUI_BITMAP bmLoading;
extern GUI_CONST_STORAGE GUI_BITMAP bmcloudrate;
extern GUI_CONST_STORAGE GUI_BITMAP bmnext1;
extern GUI_CONST_STORAGE GUI_BITMAP bmlocation;
extern GUI_CONST_STORAGE GUI_BITMAP bmBack;
extern GUI_CONST_STORAGE GUI_BITMAP bmOK;
extern GUI_CONST_STORAGE GUI_BITMAP bmCity;
extern GUI_CONST_STORAGE GUI_BITMAP bmconfirm;
extern GUI_CONST_STORAGE GUI_BITMAP bmabc;

#endif /* IMAGE_H_ */
