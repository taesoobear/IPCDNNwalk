#ifndef _IMAGEPIXEL_H_
#define _IMAGEPIXEL_H_
#pragma once

#include "Image.h"
#ifdef DrawText
#undef DrawText
#endif
class CImagePixel
{
public:
	CImagePixel();
	CImagePixel(CImage* pInput);
	~CImagePixel();
	void Init(CImage* pInput);

	inline void SetPixel(int x, int y, CPixelRGB8 color)
	{
		m_pCPP[y][x]=color;
	}

	inline CPixelRGB8& GetPixel(int x, int y) const
	{
		return m_pCPP[y][x];
	}

	inline CPixelRGB8& Pixel(int x, int y) const
	{
		return m_pCPP[y][x];
	}

	inline CPixelRGB8* operator[](int i) const	{ return m_pCPP[i];}

	void SetPixel(float x, float y, CPixelRGB8 color); ///< (0,1)좌표계에서 정의된 점을 그린다. -> 느리다는 점에 주의.
	CPixelRGB8 GetPixel(float x, float y,int& count);	///< bilinear filtering getpixel

	void DrawHorizLine(int x, int y, int width, CPixelRGB8 color);
	void DrawVertLine(int x, int y, int height, CPixelRGB8 color,bool bDotted=false);
	void DrawLine(int x1, int y1, int x2, int y2, CPixelRGB8 color);
	void DrawBox(const TRect& rect, CPixelRGB8 color);
	void DrawLineBox(const TRect& rect, CPixelRGB8 color);
	void DrawPattern(int x, int y, const CImagePixel& patternPixel, bool bUseColorKey=false, CPixelRGB8 colorkey=CPixelRGB8 (0,0,0), bool bOverideColor=false, CPixelRGB8 overrideColor=CPixelRGB8 (0,0,0));
	void DrawPattern(int x, int y, CImage* pPattern, bool bUseColorKey=false, CPixelRGB8 colorkey=CPixelRGB8 (0,0,0), bool bOverideColor=false, CPixelRGB8 overrideColor=CPixelRGB8 (0,0,0));
	void DrawSubPattern(int x, int y, const CImagePixel& patternPixel, const TRect& patternRect, bool bUseColorKey=false, CPixelRGB8 colorkey=CPixelRGB8 (0,0,0));

	void Clear(CPixelRGB8 color);
	void DrawText(int x, int y, const char* str, bool bUserColorKey=false, CPixelRGB8 colorkey=CPixelRGB8 (0,0,0));

	inline int Width() const	{ return m_pInput->GetWidth();}
	inline int Height() const	{ return m_pInput->GetHeight();}
private:
	CImage* m_pInput;
	std::vector<CPixelRGB8 *> m_pCPP;
};

class CEditableImage : public CImagePixel
{
	CImage* m_pImage;
public:
	CEditableImage (int width, int height);
	~CEditableImage ();
};
#endif
