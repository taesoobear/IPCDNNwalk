#ifndef _DRAW_CHART_H_
#define _DRAW_CHART_H_

#pragma once
class CImage;
#include "ImagePixel.h"
#include "../../BaseLib/utility/TypeString.h"
#include "../../BaseLib/math/intervals.h"
// 800 by 800 짜리 image를 만든다.
class DrawChart
{
	CImage * mpImage;
	CImagePixel mpCanvas;
	TString mXaxis;
	TString mYaxis;
	intervals mInterval;
	int x(m_real x);
	int y(m_real x);
	int canvasX(m_real x);
	int canvasY(m_real y);
	m_real paramX(int x);	// inverse of canvasX
	m_real paramY(int y);	// inverse of canvasY

public:
	DrawChart(const char* xaxis, const char* yaxis, m_real minx, m_real maxx, m_real miny, m_real maxy);
	~DrawChart(void);

	void drawMatrix(matrixn const& matrix);
	void drawScatteredData(matrixn const& matrix, CPixelRGB8 color, char* patternFile=NULL);
	void save(const char* filename);
};
#endif
