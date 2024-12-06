#ifndef _IMAGE_H_
#define _IMAGE_H_
//
// Image.H
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


typedef unsigned char uchar;

struct Int2D
{
	Int2D(){}
	Int2D(int sx, int sy):x(sx), y(sy){}
	int x;
	int y;
};

struct TRect
{
	int left, top, right, bottom;

    TRect()
    {
    }
    TRect( int l, int t, int r, int b )
    {
        left = l;
        top = t;
        right = r;
        bottom = b;
    }

	int Width() const				{ return right-left;}
	int Height() const				{ return bottom-top;}
	bool contains(Int2D pt, double margin=0)	const	{ if(pt.x>=left -margin && pt.y>=top -margin && pt.x<right +margin && pt.y<bottom+margin) return true; return false;}
	
	void enlarge(int r)				{ left-=r; right+=r; top-=r; bottom+=r;}
	int corner(Int2D pt) const
	{
		if(ABS(pt.x-left)<7 && ABS(pt.y-top)<7)
			return 1;
		if(ABS(pt.x-right)<7 && ABS(pt.y-bottom)<7)
			return 2;
		return 0;
	}
};

struct CPixelRGB8
{
	CPixelRGB8(){}
	CPixelRGB8(uchar RR, uchar GG, uchar BB):R(RR),G(GG),B(BB){}
	bool operator==(CPixelRGB8 o)	{ return (R==o.R)&& (G==o.G)&&(B==o.B);}
	uchar R;
	uchar G;
	uchar B;
};

#include "../math/template_math.h"

class CPixelsView;
class CPixels: public _tvectorn<CPixelRGB8, uchar>
{
protected:
	CPixels (uchar* ptrr, int size, int stride)
		:_tvectorn<CPixelRGB8, uchar>(ptrr,size,stride){}
public:
	CPixels ():_tvectorn<CPixelRGB8, uchar>(){}

	// 값을 카피해서 받아온다.
	template <class VecType>
	CPixels (const VecType& other)	{ assign(other);}

	explicit CPixels ( int x):_tvectorn<CPixelRGB8, uchar>() { setSize(x);}
	~CPixels(){}


	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	CPixelsView range(int start, int end, int step=1);
	const CPixelsView range(int start, int end, int step=1) const	;

	template <class VecType>
	void operator=(const VecType& other)	{ _tvectorn<CPixelRGB8, uchar>::assign(other);}

	CPixelRGB8 average() const;
};

class CPixelsView :public CPixels
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	CPixelsView (uchar* ptrr, int size, int stride):CPixels(ptrr, size, stride){}
	// 값을 reference로 받아온다.
	template <class VecType>
	CPixelsView (const VecType& other)	{ assignRef(other);}

	~CPixelsView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	void operator=(const VecType& other)	{ _tvectorn<CPixelRGB8, uchar>::assign(other);}

};


class CImage
{
private:
	Int2D _size;

public:
	uchar* _dataPtr;
	int _stride;
	uchar* _opacityMap;

	void _setDataFlipY(int width, int height, uchar* dataPtr, int stride);
	void _adoptRawData(int width, int height, uchar* dataPtr, int stride)
	{
		_size.x=width; _size.y=height;
		_dataPtr=dataPtr; _stride=stride;
		_opacityMap=NULL;
	}

public:

	CImage();
	virtual ~CImage();
	int GetWidth() const ;
	int GetHeight() const ;
	void flipY();

	// assumes RGB8 format.
	bool Create(int width, int height);

	void CopyFrom(CImage const& other);
	const uchar* GetData() const	{ return _dataPtr;}
	uchar* GetData()				{ return _dataPtr;}
	bool IsDataNull()				{ return _dataPtr==NULL;}

	/// usually NULL
	uchar* getOpacityMap() { return _opacityMap;}
	const uchar* getOpacityMap() const { return _opacityMap;}
	bool loadFromMemory(const char* filename, long data_length, const unsigned char* ptr);
	bool Load(const char* filename);
	bool Save(const char* filename) const;
	/// doesn't save opacity at the moment.
	bool save(const char* filename, int BPP) const;
	bool saveOpacity(const char* filename) const;

	CPixelsView GetHorizLine(int i)
	{
		//return CPixelsView(_dataPtr+_stride*(GetHeight()-i-1), GetWidth(), 3);
		return CPixelsView(_dataPtr+_stride*i, GetWidth(), 3);
	}

	CPixelsView GetVertLine(int i)
	{
		return CPixelsView(_dataPtr+3*i, GetHeight(), _stride);
	}

	CPixelRGB8 * GetPixel(int i, int j) const
	{
		//return (CPixelRGB8*)(_dataPtr+_stride*(GetHeight()-j-1)+i*3);
		return (CPixelRGB8*)(_dataPtr+_stride*j+i*3);
	}
	uchar * getOpacity(int i, int j) const
	{
		//return const_cast<uchar*>(_opacityMap+GetWidth()*(GetHeight()-j-1)+i);
		return const_cast<uchar*>(_opacityMap+GetWidth()*j+i);
	}

	void GetVertLine(int i, CPixels& out);


};

vector3 rgb2hsv(vector3 const& in); // r,g,b : a fraction between 0 and 1
vector3 hsv2rgb(vector3 const& in); // h : angle in degrees, s,v : a fraction between 0 and 1
#endif
