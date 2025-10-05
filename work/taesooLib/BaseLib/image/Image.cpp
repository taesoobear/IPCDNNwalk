//
// Image.cpp
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

// Image.cpp: implementation of the CImage class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Image.h"
#ifndef NO_FREEIMAGE
#include "FreeImage.h"
#endif
CPixelsView CPixels::range(int start, int end, int step)
{
	return _range<CPixelsView >(start,end,step);
}

const CPixelsView CPixels::range(int start, int end, int step) const
{
	return ((CPixels*)this)->range(start, end, step);
}

CPixelRGB8 CPixels::average() const
{
	long sumR=0, sumG=0, sumB=0;

	for(int i=0; i<size(); i++)
	{
		sumR+=(long)value(i).R;
		sumG+=(long)value(i).G;
		sumB+=(long)value(i).B;
	}
	sumR/=size();
	sumG/=size();
	sumB/=size();
	return CPixelRGB8(sumR, sumG, sumB);
}

;

CImage::CImage()
	:
	_size(0,0),
	_dataPtr(NULL),
	_opacityMap(NULL),
	_stride(0)
{
}

CImage::~CImage()
{
	if(_dataPtr)
	{
		delete [] _dataPtr;
		delete[] _opacityMap;
		_dataPtr=NULL;
		_opacityMap=NULL;
	}
}

int CImage::GetWidth() const
{return _size.x;}
int CImage::GetHeight()const
{return _size.y;}

void CImage::CopyFrom(CImage const& other)
{


	_size.x=other.GetWidth();
	_size.y=other.GetHeight();
	_dataPtr=new uchar[_size.x*_size.y*3];
	_stride=GetWidth()*3;
	for(int y=0; y<_size.y; y++)
		memcpy(&_dataPtr[y*_stride], &other._dataPtr[y*other._stride], _stride);

	if(other._opacityMap)
	{
		_opacityMap=new uchar[_size.x*_size.y];
		for(int y=0; y<_size.y; y++)
			memcpy(&_dataPtr[y*_stride], &other._dataPtr[y*other._stride], GetWidth());
	}
	else
		_opacityMap=NULL;
}

void CImage::_setDataFlipY(int width, int height, uchar* dataPtr, int stride)
{
	_size.x=width;
	_size.y=height;
	_dataPtr=new uchar[width*height*3];
	_stride=GetWidth()*3;
	for(int y=0; y<_size.y; y++)
		//memcpy(&_dataPtr[y*_stride], &dataPtr[(_size.y-y-1)*stride], _stride); // flipY
		memcpy(&_dataPtr[y*_stride], &dataPtr[y*stride], _stride); // no flipY
	_opacityMap=NULL;
}

// assumes RGB8 format.
bool CImage::Create(int width, int height)
{
	_size.x=width;
	_size.y=height;
	_dataPtr=new uchar[width*height*3];
	_stride=GetWidth()*3;
	memset((void*)_dataPtr, 200, width*height*3);
	_opacityMap=NULL;
	return true;
}

void CImage::GetVertLine(int i, CPixels& out)
{
	out.setSize(GetHeight());

	for(int j=0; j<GetHeight(); j++)
	{
		out[j]=*GetPixel(i,j);
	}
}

static void CImage_flipY(CImage& inout)
{
	CPixelRGB8 * line_swap=new CPixelRGB8[inout.GetWidth()];
	for(int i=0; i<inout.GetHeight()/2; i++)
	{
		CPixelRGB8* line1=inout.GetPixel(0,i);
		CPixelRGB8* line2=inout.GetPixel(0,inout.GetHeight()-i-1);
		memcpy(line_swap, line1, sizeof(CPixelRGB8)*inout.GetWidth());
		memcpy(line1, line2, sizeof(CPixelRGB8)*inout.GetWidth());
		memcpy(line2, line_swap, sizeof(CPixelRGB8)*inout.GetWidth());
	}
	delete[] line_swap;

	if(inout.getOpacityMap()) 
	{
		unsigned char * line_swap=new unsigned char[inout.GetWidth()];
		for(int i=0; i<inout.GetHeight()/2; i++)
		{
			unsigned char* line1=inout.getOpacity(0,i);
			unsigned char* line2=inout.getOpacity(0, inout.GetHeight()-i-1);
			memcpy(line_swap, line1, sizeof(unsigned char)*inout.GetWidth());
			memcpy(line1, line2, sizeof(unsigned char)*inout.GetWidth());
			memcpy(line2, line_swap, sizeof(unsigned char)*inout.GetWidth());
		}
		delete[] line_swap;
	}
}
void CImage::flipY()
{
	CImage_flipY(*this);
}
static void setData(unsigned char* _dataPtr, unsigned char*& _opacityMap, Int2D& _size, const unsigned char* pixeles)
{
	bool hasTransparency=false;
	int transparencyCount=0;
	for(int j= 0; j<_size.x*_size.y; j++)
	{
		_dataPtr[j*3+0]= pixeles[j*4+2];
		_dataPtr[j*3+1]= pixeles[j*4+1];
		_dataPtr[j*3+2]= pixeles[j*4+0];
		_opacityMap[j]= pixeles[j*4+3];
		transparencyCount+=int (_opacityMap[j]<250);
	}

	// 현재 alpha채널이 있으면 depth test를 끄는 코드가 fbximporter에 있어서.
	// conservative하게 25% 이상 투명 픽셀이 있을때만 alpha채널 살렸음. 
	if (transparencyCount>((_size.x*_size.y)/4))
		hasTransparency=true;

	if(!hasTransparency)
	{
		delete[]_opacityMap;
		_opacityMap=NULL;
	}
}

#if 1 // turn off timer
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)
#define END_TIMER2(x)
#else
#include "../utility/QPerformanceTimer.h"
#endif
bool CImage::loadFromMemory(const char* filename, long data_length, const unsigned char* ptr)
{

#ifndef NO_FREEIMAGE
	BEGIN_TIMER(freeImageLoad);
	FREE_IMAGE_FORMAT formato = FreeImage_GetFileType(filename,0);
	if(formato == FIF_UNKNOWN) {
		// no signature ?
		// try to guess the file format from the file extension
		formato = FreeImage_GetFIFFromFilename(filename);
	}
	FIMEMORY * stream=FreeImage_OpenMemory(const_cast<unsigned char*>(ptr), data_length);
	FIBITMAP* imagen = FreeImage_LoadFromMemory(formato, stream);
	if (!imagen)
		printf("error??? loading(mem) %s %d %d\n", filename, data_length, formato);
	if ( FreeImage_GetBPP( imagen ) != 32 )
	{
		FIBITMAP* temp = imagen;  
		imagen = FreeImage_ConvertTo32Bits(imagen);
		FreeImage_Unload(temp);
	}
	FreeImage_CloseMemory(stream);
	_size.x = FreeImage_GetWidth(imagen);
	_size.y = FreeImage_GetHeight(imagen);
	if(_size.x==0)
		printf("resolution error!! %d %d, %s\n", _size.x, _size.y, filename);
	_dataPtr=new unsigned char[3*_size.x*_size.y];
	_opacityMap=new unsigned char[_size.x*_size.y];
	_stride=GetWidth()*3;

	unsigned char* pixeles = (unsigned char*)FreeImage_GetBits(imagen);
	setData(_dataPtr, _opacityMap, _size, pixeles);
	FreeImage_Unload(imagen);

#else
	ASSERT(false);
	printf("error!no freeimage\n");
#endif

	END_TIMER2(freeImageLoad2);

	//Save("out.jpg");
	return true;
}
#ifndef NO_FREEIMAGE
static void FreeImageErrorHandler(FREE_IMAGE_FORMAT fif, const char *message) {
 printf("\n*** "); 
 if(fif != FIF_UNKNOWN) {
 printf("%s Format\n", FreeImage_GetFormatFromFIF(fif));
 }
 printf(message);
 printf(" ***\n");
}
#endif
// In your main program …
bool CImage::Load(const char* filename)
{
#ifndef NO_FREEIMAGE
	FreeImage_SetOutputMessage(FreeImageErrorHandler);
	FREE_IMAGE_FORMAT formato = FreeImage_GetFileType(filename,0);
	if(formato == FIF_UNKNOWN) {
		// no signature ?
		// try to guess the file format from the file extension
		formato = FreeImage_GetFIFFromFilename(filename);
	}
	FIBITMAP* imagen = FreeImage_Load(formato, filename);
	if (!imagen)
		printf("error??? loading %s\n", filename);
	if ( FreeImage_GetBPP( imagen ) != 32 )
	{
		FIBITMAP* temp = imagen;  
		imagen = FreeImage_ConvertTo32Bits(imagen);
		FreeImage_Unload(temp);
	}

	_size.x = FreeImage_GetWidth(imagen);
	_size.y = FreeImage_GetHeight(imagen);
	if(_size.x==0)
		printf("resolution error!! %d %d, %s\n", _size.x, _size.y, filename);
	_dataPtr=new unsigned char[3*_size.x*_size.y];
	_opacityMap=new unsigned char[_size.x*_size.y];
	_stride=GetWidth()*3;

	unsigned char* pixeles = (unsigned char*)FreeImage_GetBits(imagen);
	setData(_dataPtr, _opacityMap, _size, pixeles);
	FreeImage_Unload(imagen);
#else
	printf("error! no freeimage\n");
#endif


	//Save("out.jpg");
	return true;
}


namespace Imp
{
	void DeleteFile( const char* filename);
}

namespace Imp
{
	bool IsFileExist(const char* filename);
}

#ifndef NO_FREEIMAGE
//#pragma comment(lib, "FreeImage/Dist/FreeImage.lib")

/** Generic image writer
@param dib Pointer to the dib to be saved
@param lpszPathName Pointer to the full file name
@param flag Optional save flag constant
@return Returns true if successful, returns false otherwise
*/
bool GenericWriter(FIBITMAP* dib, const char* lpszPathName, int flag=0) {
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	BOOL bSuccess = FALSE;

	if(dib) {
		// try to guess the file format from the file extension
		fif = FreeImage_GetFIFFromFilename(lpszPathName);
		if(fif != FIF_UNKNOWN ) {
			// check that the plugin has sufficient writing and export capabilities ...
			WORD bpp = FreeImage_GetBPP(dib);
			if(FreeImage_FIFSupportsWriting(fif) && FreeImage_FIFSupportsExportBPP(fif, bpp)) {
				// ok, we can save the file
				bSuccess = FreeImage_Save(fif, dib, lpszPathName, flag);
				// unless an abnormal bug, we are done !
			}
		}
	}
	return (bSuccess == TRUE) ? true : false;
}


static void CImage_SaveFreeImage(CImage & image, const char* filename, int BPP=-1)
{
//	applyFloydSteinberg(image, 4);

	BYTE* bits=image._dataPtr;
	int width=image.GetWidth();
	int height=image.GetHeight();
	int scan_width=image._stride;

	{
		BYTE* pixels=bits;
		int w=width;
		int h=height;
		unsigned char temp;
		int pos;
		//---------------------------- swap RGB to BGR
		for (int i = 0; i < w*h; i++){
			pos=i*3;
			temp = pixels[pos];
			pixels[pos  ] = pixels[pos+2];
			pixels[pos+2] = temp;
		}
	}
	// convert a 32-bit raw buffer (top-left pixel first) to a FIBITMAP
	// ----------------------------------------------------------------
	FIBITMAP *dst = FreeImage_ConvertFromRawBits(bits, width, height, scan_width,
			24, 0, 0, 0, FALSE);
	{
		BYTE* pixels=bits;
		int w=width;
		int h=height;
		unsigned char temp;
		int pos;
		//---------------------------- back to RGB
		for (int i = 0; i < w*h; i++){
			pos=i*3;
			temp = pixels[pos];
			pixels[pos  ] = pixels[pos+2];
			pixels[pos+2] = temp;
		}
	}


	
	TString ext=TString(filename).right(3).toUpper();

	FIBITMAP *dst_grey=NULL;
	FIBITMAP *dst2=NULL;
	
	if(ext=="BMP")
	{
		if (BPP==-1) BPP=4;
	}
	else if(ext=="GIF")
	{
		if (BPP==-1) BPP=8;
	}
	else if(BPP==-1) 
		BPP=24;

	if (BPP==4)
	{
		dst_grey=FreeImage_ConvertToGreyscale(dst);
		dst2 = FreeImage_ConvertTo4Bits(dst_grey);
		GenericWriter(dst2, filename);
		FreeImage_Unload(dst_grey);
		FreeImage_Unload(dst2);
	}
	else if(BPP==8)
	{
		dst_grey=FreeImage_ConvertToGreyscale(dst);		
		GenericWriter(dst_grey, filename);
		FreeImage_Unload(dst_grey);
	}
	else
		GenericWriter(dst, filename);

	//FIBITMAP *dst2 = FreeImage_Dither(dst, FID_FS);
	FreeImage_Unload(dst);
}
#endif

bool CImage::Save(const char* filename) const
{
	/*
	if(Imp::IsFileExist(filename))
	{
	//	if(!Msg::confirm("Do you want to overwrite file %s?", filename))
	//		return false;
		deleteFile(filename);
	}

	TString ext=TString(filename).right(3).toUpper();
	if(ext=="BMP" || ext=="GIF")
	{
		CImage_SaveFreeImage(*this, filename);
	}
	else
	{
		if (TString(filename).right(3).toUpper()=="JPG")
		{
			printf("flipping..\n");

		//if (_flipped)
			CImage_flipY(*this);
		}
		ilBindImage(_id);
		ilSaveImage(filename);
		if (TString(filename).right(3).toUpper()=="JPG")
		//if (_flipped)
			CImage_flipY(*this);
	}
	*/
	save(filename, 24);
	return 1;
}

bool CImage::saveOpacity(const char* filename) const
{
	if(!getOpacityMap())
		return false;

#ifndef NO_FREEIMAGE
	BYTE* bits=const_cast<BYTE*>(getOpacityMap());
	int width=GetWidth();
	int height=GetHeight();
	int scan_width=width;
	
	// convert a 8-bit raw buffer (top-left pixel first) to a FIBITMAP
	// ----------------------------------------------------------------
	FIBITMAP *dst = FreeImage_ConvertFromRawBits(bits, width, height, scan_width,
			8, 0, 0, 0, FALSE);

	GenericWriter(dst, filename);

	//FIBITMAP *dst2 = FreeImage_Dither(dst, FID_FS);
	FreeImage_Unload(dst);
#endif
}
bool CImage::save(const char* filename, int BPP) const
{
	if(Imp::IsFileExist(filename))
	{
//		if(!Msg::confirm("Do you want to overwrite file %s?", filename))
//			return false;
		deleteFile(filename);
	}

#ifndef NO_FREEIMAGE
	CImage_SaveFreeImage((CImage&)(*this), filename, BPP);
#endif
	return true;
}

/**
* @author Ralph Hauwert
* 
* The ImageDithering class allows for simple image color quantization and dithering.
* It aims to offer some well-known spatial dithering algorithms for the AS3 Language.
* 
* ralph@unitzeroone.com
* http://www.unitzeroone.com
*/

/**
* applyFloydSteinberg(bitmapData:BitmapData, levels:int);
* 
* Floyd-Steinberg dithering is an image dithering algorithm first published in 1976 by Robert W. Floyd and Louis Steinberg.
* Using error diffusion, this form of dithering is visually much better then ordered diffusion like Bayer's.
* R.W. Floyd, L. Steinberg, "An adaptive algorithm for spatial grey scale". Proceedings of the Society of Information Display 17, 75??7 (1976).
*/
#include "ImagePixel.h"
void gammaCorrection(CImage& _bitmapData, double fGamma)
{
	unsigned char gammaTable[256];

	int lr;
	for(int i=0; i<256; i++)
	{
		int lr=int( pow( ((double)i)/255.0, fGamma)*255.0+0.5);
		lr=	lr < 0 ? lr = 0: lr > 255 ? lr = 255 : lr;
		gammaTable[i]= (unsigned char)lr;
	}
	CImagePixel bitmapData(&_bitmapData);

	for(int y=0; y<bitmapData.Height();y++)
	{
		CPixelRGB8* line=bitmapData[y];

		for(int x=0; x<bitmapData.Width();x++)
		{
			line[x].R=gammaTable[line[x].R];
			line[x].G=gammaTable[line[x].G];
			line[x].B=gammaTable[line[x].B];
		}		
	}
}

void applyFloydSteinberg(CImage& _bitmapData, int _levels)
{
	double levels=(double)_levels;
	int lNorm= 256/_levels;
	int hlNorm=lNorm/2;
	//The FS kernel...note the 16th. Optimisation can still be done.
	double d1= 7/16.0;
	double d2= 3/16.0;
	double d3= 5/16.0;
	double d4= 1/16.0;

	int lc, r, g, b, nr, ng, nb, er, eg, eb, lr, lg, lb;
	int x=0;
	int y=0;

	int lNorm2=256/(_levels+1);
	int hlNorm2=lNorm2/2;
	CImagePixel bitmapData(&_bitmapData);

//#define USE_DEC_DYNRANGE
#ifdef USE_DEC_DYNRANGE
#define DEC_DYN(x)	(x)*lNorm2/lNorm+hlNorm2;
	for(y=0; y<bitmapData.Height();y++)
	{
		CPixelRGB8* line=bitmapData[y];

		for(x=0; x<bitmapData.Width();x++)
		{
			line[x].R=DEC_DYN(int(line[x].R))
			line[x].G=DEC_DYN(int(line[x].G))
			line[x].B=DEC_DYN(int(line[x].B))
		}		
	}
	
#endif

#define QUANTIZE2(x) (x)/lNorm*lNorm+hlNorm;

	for(y=0; y<bitmapData.Height()-1;y++)
	{
		CPixelRGB8* line=bitmapData[y];
		CPixelRGB8* nline=bitmapData[y+1];

		for(x=1; x<bitmapData.Width()-1;x++)
		{
			//Retrieve current RGB value.
			CPixelRGB8& c = line[x];
			r = c.R;
			g = c.G;
			b = c.B;

			//Normalize and scale to the number of levels.
			nr = QUANTIZE2(r);
			ng = QUANTIZE2(g);
			nb = QUANTIZE2(b);

			//Set the current pixel.
			c.R=nr;
			c.G=ng;
			c.B=nb;

			//Quantization error.
			er = (r)-nr;
			eg = (g)-ng;
			eb = (b)-nb;

			//Apply the kernel.
			//+1,0
			{
				CPixelRGB8& lc = line[x+1];
				lr = (int)lc.R + (d1*er);
				lg = (int)lc.G + (d1*eg);
				lb = (int)lc.B + (d1*eb);

				//Clip & Set
				lr < 0 ? lr = 0: lr > 255 ? lr = 255 : lr;
				lg < 0 ? lg = 0: lg > 255 ? lg = 255 : lg;
				lb < 0 ? lb = 0: lb > 255 ? lb = 255 : lb;

				lc.R=lr;
				lc.G=lg;
				lc.B=lb;
			}

			//-1,+1
			{
				CPixelRGB8& lc = nline[x-1];
				lr = (int)lc.R + (d2*er);
				lg = (int)lc.G + (d2*eg);
				lb = (int)lc.B + (d2*eb);

				//Clip & Set
				lr < 0 ? lr = 0: lr > 255 ? lr = 255 : lr;
				lg < 0 ? lg = 0: lg > 255 ? lg = 255 : lg;
				lb < 0 ? lb = 0: lb > 255 ? lb = 255 : lb;
				
				lc.R=lr;
				lc.G=lg;
				lc.B=lb;
			}

			//0,+1
			{
				CPixelRGB8& lc = nline[x];
				lr = (int)lc.R + (d3*er);
				lg = (int)lc.G + (d3*eg);
				lb = (int)lc.B + (d3*eb);

				//Clip & Set
				lr < 0 ? lr = 0: lr > 255 ? lr = 255 : lr;
				lg < 0 ? lg = 0: lg > 255 ? lg = 255 : lg;
				lb < 0 ? lb = 0: lb > 255 ? lb = 255 : lb;

				lc.R=lr;
				lc.G=lg;
				lc.B=lb;
			}

			//+1,+1
			{
				CPixelRGB8& lc = nline[x+1];
				lr = (int)lc.R + (d4*er);
				lg = (int)lc.G + (d4*eg);
				lb = (int)lc.B + (d4*eb);

				//Clip & Set
				lr < 0 ? lr = 0: lr > 255 ? lr = 255 : lr;
				lg < 0 ? lg = 0: lg > 255 ? lg = 255 : lg;
				lb < 0 ? lb = 0: lb > 255 ? lb = 255 : lb;

				lc.R=lr;
				lc.G=lg;
				lc.B=lb;
			}
		}
	}

	for(y=0; y<bitmapData.Height();y++)
	{
		CPixelRGB8* line=bitmapData[y];

		for(x=0; x<bitmapData.Width();x++)
		{
			CPixelRGB8& c = line[x];
			r = c.R;
			g = c.G;
			b = c.B;

			//Normalize and scale to the number of levels
			// : quantization
			nr = QUANTIZE2(r);
			ng = QUANTIZE2(g);
			nb = QUANTIZE2(b);

			c.R=nr;
			c.G=ng;
			c.B=nb;
		}
	}
}

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

static hsv   rgb2hsv(rgb in);
static rgb   hsv2rgb(hsv in);

vector3 rgb2hsv(vector3 const& _in)
{
	rgb in;
	in.r=_in.x;
	in.g=_in.y;
	in.b=_in.z;
	hsv out=rgb2hsv(in);
	return vector3(out.h, out.s, out.v);
}
vector3 hsv2rgb(vector3 const& _in)
{
	hsv in;
	in.h=_in.x;
	in.s=_in.y;
	in.v=_in.z;
	rgb out=hsv2rgb(in);
	return vector3(out.r, out.g, out.b);
}

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}
