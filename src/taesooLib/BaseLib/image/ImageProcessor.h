#ifndef _IMAGEPROCESSOR_H_
#define _IMAGEPROCESSOR_H_
//
// ImageProcessor.h
//
// Copyright 2004 by Taesoo Kwon.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA.
//

#pragma once

#include "Image.h"

// for backward compatibility
#define CImageProcessor Imp
namespace Imp
{
	void drawBox(CImage& inout, TRect const& t, int R, int G, int B);
	void sharpen(CImage& inout, double factor, int iterations);
	void contrast(CImage& inout, double factor);
	void gammaCorrect(CImage& inout, double factor);
	void dither(CImage& inout, int levels);
	void resize(CImage& inout, int width, int height);
	void blit(CImage& out, CImage const& in, TRect const& rect_in, int x, int y);
	void concatVertical(CImage& out, CImage const& a, CImage const& b);
	void crop(CImage& out, CImage const& in, int left, int top, int right, int bottom);
	void rotateRight(CImage& other);
	void rotateRight(CImage& out, CImage const& in);
	void rotateLeft(CImage& other);
	void rotateLeft(CImage& out, CImage const& in);

	CImage* Clone(CImage* pInput);

	enum {LINE_CHART, BAR_CHART};
	void SaveAndDeleteImage(CImage* pImage, const char* filename);

	CPixelRGB8 GetColor(int i);

	CImage* DrawChart(const vectorn& vector, int chart_type, float min=0, float max=0); 	//!< 0 frame부터 n-1프레임까지의 float value를 그린다.
	CImage* DrawChart(const matrixn& matrix, int chart_type, vectorn const& aMin, vectorn const& aMax, double* aY=NULL);	//!< 0 frame부터 n-1프레임까지의 여러 signal들을 그린다.
	CImage* DrawChart(const matrixn& matrix, int chart_type, float min=0, float max=0, float horizLine=FLT_MAX);		//!< 0 frame부터 n-1프레임까지의 여러 signal들을 그린다.
	CImage* DrawChart(const matrixn& matrix, float min=0, float max=0);						//!< 0 frame부터 n-1프레임까지의 멀티 dimensional signal을 그린다.
	CImage* DrawChart(const bitvectorn& ab, CPixelRGB8 color);
	CImage* DrawChart(const intvectorn& ab, const char* colormapfile="Resource/default/colormap.bmp");
	CImage* DrawChartText(const intvectorn& ab, TStrings* translationTable=NULL);
	void ChangeChartPrecision(int precision);
	void DefaultPrecision();

	// private
	namespace _private
	{
		extern int g_nChartPrecision;
		void DrawChart(CImage* pInput, int numFrame, double* aValue,float min, float max, CPixelRGB8 color, int xoffset, int yoffset, int xdelta,int chart_type);
	}
}

#endif
