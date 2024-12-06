//
// ImageProcessor.cpp
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

// ImageProcessor.cpp: implementation of the CImageProcessor class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ImageProcessor.h"
#include "Image.h"
#ifndef NO_DEVIL
#include <IL/il.h>
#include <IL/ilu.h>
#endif
#include "ImagePixel.h"
#include "../math/mathclass.h"
#include "../math/bitVectorN.h"

namespace Imp{
void overlay(CImage& out, CImage const& in, int out_x, int out_y)
{
	int right=MIN(out.GetWidth(), in.GetWidth()+out_x);
	int bottom=MIN(out.GetHeight(), in.GetHeight()+out_y);
	
	//printf("%d %d %d %d\n", in.GetWidth(), in.GetHeight(), out.GetWidth(), out.GetHeight());
	for(int j=out_y; j<bottom; j++)
	{
		//printf("j:%d %d\n", j, out_y);
		CPixelRGB8 * line=in.GetPixel(0, j-out_y);
		//printf("a\n");
		CPixelRGB8 * outl=out.GetPixel(0, j);
		//printf("b %d %d %d\n", out_x, right, in.GetWidth());
		for(int i=out_x; i<right; i++)
		{
			outl[i]=line[i-out_x];
		}
		//printf("c %d %d\n", out.GetWidth(), out.GetHeight());
	}
}
}

void Imp::blit(CImage& out, CImage const& in, TRect const& _rect_in, int x, int y)
{
	TRect rect_in=_rect_in;

	if (rect_in.right>in.GetWidth())
		rect_in.right=in.GetWidth();
	if (rect_in.bottom>in.GetHeight())
		rect_in.bottom=in.GetHeight();
	if (rect_in.bottom<=rect_in.top)
	{
		printf("%d %d %d\n", rect_in.bottom, rect_in.top, in.GetHeight());
		return;
	}

	if(&out==&in)
	{
		CImage src;
		src.CopyFrom(in);
		blit(out, src, _rect_in, x, y);
		return;
	}

	//out.Create(rect_in.right-rect_in.left, rect_in.bottom-rect_in.top);
	//printf("0\n");
	RANGE_ASSERT(&out != &in);
	//printf("1\n");
	RANGE_ASSERT(rect_in.left>=0);
	//printf("2\n");
	RANGE_ASSERT(rect_in.right<=in.GetWidth());
	//printf("3\n");
	RANGE_ASSERT(rect_in.top>=0);
	//printf("5\n");
	RANGE_ASSERT(rect_in.right>rect_in.left);
	//printf("6\n");
	RANGE_ASSERT(rect_in.bottom>rect_in.top);
	//printf("7\n");
	RANGE_ASSERT(x<out.GetWidth());
	//printf("8\n");
	RANGE_ASSERT(y<out.GetHeight());
	//printf("9\n");
	RANGE_ASSERT(x+rect_in.Width()<=out.GetWidth());
	//printf("1\n");
	//RANGE_ASSERT(y+rect_in.Height()<=out.GetHeight());
	
	for(int j=rect_in.top; j<rect_in.bottom; j++)
	{
		if(j-rect_in.top+y>=out.GetHeight()) break;
		CPixelRGB8 * line=in.GetPixel(0, j);
		CPixelRGB8 * outl=out.GetPixel(0, j-rect_in.top+y);
		for(int i=rect_in.left; i<rect_in.right; i++)
		{
			outl[i-rect_in.left+x]=line[i];
		}
	}
	//printf("2\n");
}

void Imp::downsample4(CImage& out, CImage const& in)
{
	int newWidth=in.GetWidth()/2;
	int newHeight=in.GetHeight()/2;
	out.Create(newWidth, newHeight);
	CImagePixel op(&out);
	CImagePixel ip((CImage*)&in);
	
	for(int i=0; i<newHeight; i++)
	{
		CPixelRGB8* lineo=out.GetPixel(0,i);
		CPixelRGB8* line1=in.GetPixel(0,i*2);
		CPixelRGB8* line2=in.GetPixel(0,i*2+1);
		for(int j=0; j<newWidth; j++)
		{
			CPixelRGB8& p1=line1[j*2];
			CPixelRGB8& p2=line1[j*2+1];
			CPixelRGB8& p3=line2[j*2];
			CPixelRGB8& p4=line2[j*2+1];
			
			lineo[j].R=(unsigned char)(((int)p1.R+(int)p2.R+(int)p3.R+(int)p4.R+2)/4);
			lineo[j].G=(unsigned char)(((int)p1.G+(int)p2.G+(int)p3.G+(int)p4.G+2)/4);
			lineo[j].B=(unsigned char)(((int)p1.B+(int)p2.B+(int)p3.B+(int)p4.B+2)/4);
		}
	}
}
void Imp::drawBox(CImage& inout, TRect const& t, int R, int G, int B)
{
	CImagePixel p(&inout);
	p.DrawBox(t, CPixelRGB8(R,G,B));
}
void Imp::concatVertical(CImage& out, CImage const& a, CImage const& b)
{
	//ASSERT(a.GetWidth()==b.GetWidth());

	if(a.GetHeight()==0)
	{
		ASSERT(b.GetHeight()!=0);
		out.CopyFrom(b);
		return;
	}

	if(&a==&out)
	{
		CImage temp;
		temp.CopyFrom(a);
		concatVertical(out, temp, b);
		return;
	}

	if(&b==&out)
	{
		CImage temp;
		temp.CopyFrom(b);
		concatVertical(out, a, temp);
		return;
	}

	out.Create(MAX(a.GetWidth(), b.GetWidth()), a.GetHeight()+b.GetHeight()+1);
	
	if(a.GetWidth()<b.GetWidth())
	{
		CImagePixel p(&out);
		p.DrawBox(TRect(a.GetWidth(), 0, b.GetWidth(), a.GetHeight()), CPixelRGB8(255,255,255));
	}
	else if(b.GetWidth()<a.GetWidth())
	{
		CImagePixel p(&out);
		p.DrawBox(TRect(b.GetWidth(),a.GetHeight(), a.GetWidth(), out.GetHeight()), CPixelRGB8(255,255,255));
	}
#ifndef NO_DEVIL
	ilBindImage(out._getILid());
	ilOverlayImage(a._getILid(), 0,0, 0);

	CImagePixel p(&out);
	p.DrawHorizLine(0, a.GetHeight(), out.GetWidth(), CPixelRGB8(255,255,255));
	p.DrawHorizLine(10, a.GetHeight(), 10, CPixelRGB8(128,128,128));

	ilOverlayImage(b._getILid(), 0, a.GetHeight()+1, 0);
#else
	overlay(out, a, 0,0);
	CImagePixel p(&out);
	p.DrawHorizLine(0, a.GetHeight(), out.GetWidth(), CPixelRGB8(255,255,255));
	p.DrawHorizLine(10, a.GetHeight(), 10, CPixelRGB8(128,128,128));

	overlay(out, b, 0,a.GetHeight()+1);
#endif
}

void Imp::sharpen(CImage& out, double factor, int iterations)
{
#ifndef NO_DEVIL

	ilBindImage(out._getILid());
	iluSharpen(factor, iterations);
	#endif
}

void Imp::contrast(CImage& out, double factor)
{
#ifndef NO_DEVIL
	ilBindImage(out._getILid());
	iluContrast(factor);
#endif
}

void gammaCorrection(CImage& _bitmapData, double fGamma);
void Imp::gammaCorrect(CImage& out, double factor)
{
		// my implementation
		gammaCorrection(out, 1.0/factor);
}

void applyFloydSteinberg(CImage& _bitmapData, int _levels);

void Imp::dither(CImage& out, int levels)
{
	applyFloydSteinberg(out, levels);
}

void Imp::rotateRight(CImage& other)
{
	CImage src;
	src.CopyFrom(other);
	Imp::rotateRight(other, src);
}

void Imp::rotateLeft(CImage& other)
{
	CImage src;
	src.CopyFrom(other);
	Imp::rotateLeft(other, src);
}
void Imp::rotateLeft(CImage& out, CImage const& in)
{
	CImage* pOutput=&out;
	CImage* pInput=(CImage*)&in;
	int width=pInput->GetWidth();
	int height=pInput->GetHeight();

	pOutput->Create(height,width);

	CImagePixel inputptr(pInput);
	CImagePixel outputptr(pOutput);

	for(int i=0; i<width; i++)
	{
		for(int j=0; j<height; j++)
		{
			outputptr[width-i-1][j]=inputptr[j][i];
		}
	}
}
void Imp::rotateRight(CImage& out, CImage const& in)
{
	CImage* pOutput=&out;
	CImage* pInput=(CImage*)&in;
	int width=pInput->GetWidth();
	int height=pInput->GetHeight();

	pOutput->Create(height,width);

	{
		CImagePixel inputptr(pInput);
		CImagePixel outputptr(pOutput);

		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[i][height-j-1]=inputptr[j][i];
			}
		}
	}
}


void Imp::crop(CImage& out, CImage const& in, int left, int top, int right, int bottom)
{
	if(&out==&in)
	{
		CImage src;
		src.CopyFrom(in);
		crop(out, src, left, top, right, bottom);
		return;
	}
	int width=right-left;
	int height=bottom-top;
	out.Create(width, height);
	Imp::blit(out, in, TRect(left, top, right, bottom), 0,0);
#if 0

	out.Create(width, height);
	ilBindImage(out._getILid());
	ilBlit(in._getILid(), 0,0,0, left, top, 0, width, height, 1);
	#endif
}





int Imp::_private::g_nChartPrecision=256;
int Imp::_private::g_nBoolChartPrecision=10;

void Imp::ChangeChartPrecision(int precision)
{
	Imp::_private::g_nChartPrecision=precision;
}
void Imp::ChangeBoolChartPrecision(int precision)
{
	Imp::_private::g_nBoolChartPrecision=precision;
}

void Imp::DefaultPrecision()
{
	Imp::_private::g_nChartPrecision=256;
}

namespace Imp
{

	void DeleteFile(const char* filename);
bool IsFileExist(const char* filename)
{
	FILE* temp;
	temp=fopen(filename,"r");
	if(!temp) return false;
	fclose(temp);
	return true;
}


void SaveAndDeleteImage(CImage* pImage, const char* filename)
{
	if(pImage==NULL)
		return;

	if(IsFileExist(filename))
		deleteFile(filename);
	pImage->Save(filename);
	delete pImage;
}



void Crop(CImage* pOutput, CImage *pInput, const TRect &rect)
{
	ASSERT(pOutput);
	int width=rect.Width()+1;
	int height=rect.Height()+1;

	RANGE_ASSERT(pOutput->GetWidth()==width);
	RANGE_ASSERT(pOutput->GetHeight()==height);

	TRect inputRect(0,0, pInput->GetWidth(), pInput->GetHeight());
	Int2D point;

	{
		CImagePixel inputptr(pInput);
		CImagePixel outputptr(pOutput);

		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				point.x=i+rect.left;
				point.y=j+rect.top;

				if(inputRect.contains(point))
					outputptr[j][i]=inputptr[j+rect.top][i+rect.left];
				else
				{
					outputptr[j][i].R=0;
					outputptr[j][i].G=0;
					outputptr[j][i].B=0;
				}

			}
		}
	}

}

CImage* Clone(CImage* pInput)
{
	CImage* pOutput=new CImage();
	pOutput->Create(pInput->GetWidth(), pInput->GetHeight());
	Crop(pOutput, pInput, TRect(0,0,pInput->GetWidth()-1, pInput->GetHeight()-1));
	return pOutput;
}


int ToImageY(double y, double min, double max, int yoffset)
{
	#define M_ToImageY(x) (int(((x)-min)/(max-min)*((double)_private::g_nChartPrecision-1.f)+0.5f))
	int j=M_ToImageY(y);
	if(j<0) j=0;
	else if(j>(_private::g_nChartPrecision-1)) j=(_private::g_nChartPrecision-1);
	//j=(_private::g_nChartPrecision-1)-j; // flipY
	j+=yoffset;
	return j;
}

void _private::DrawChart(CImage* pInput, int numFrame, double* aValue,float min, float max, CPixelRGB8 color, int xoffset, int yoffset, int xdelta,int chart_type)
{
	ASSERT(pInput->GetWidth() >= numFrame);
	ASSERT(pInput->GetHeight() >= _private::g_nChartPrecision+yoffset);

	// min이하는 pixel (_private::g_nChartPrecision-1)에 그려지고, max이상은 0에 그려진다. 즉 아래쪽이 min값

	CImagePixel inputptr(pInput);

	if(min<=0 && max>=0)
	{
		// draw y=0 line.
		int zero_y=ToImageY(0, min, max, yoffset);
		inputptr.DrawLine(0+xoffset, zero_y, xdelta*(numFrame-1)+xoffset,zero_y,CPixelRGB8(208,208,208));
	}
	// draw top line
	inputptr.DrawLine( 0+xoffset, (_private::g_nChartPrecision-1)+yoffset, xdelta*(numFrame-1)+xoffset, (_private::g_nChartPrecision-1)+yoffset, CPixelRGB8(128,128,128));

	char temp[100];
    sprintf(temp,"%.2f", min);
	inputptr.DrawText(0+xoffset,yoffset, temp);
	sprintf(temp,"%.2f", max);
	inputptr.DrawText(0+xoffset,pInput->GetHeight()-20+yoffset, temp);



	vector3 vcolor;
	vcolor.x=(color.R);
	vcolor.y=(color.G);
	vcolor.z=(color.B);

	vector3 white(255,255,255);
	vector3 hcolor;
	hcolor.interpolate(0.7, vcolor, white);

	CPixelRGB8 halfcolor=CPixelRGB8(int(hcolor.x), int(hcolor.y), int(hcolor.z));


	int prev_x=-1, prev_y;
	double* itr;
	for(int i=0; i<numFrame; i++)
	{
		itr=&aValue[i];

		int j=ToImageY(*itr, min, max, yoffset);
		if(chart_type==LINE_CHART)
		{
			if(prev_x!=-1)
				inputptr.DrawLine( xdelta*prev_x+xoffset, prev_y, xdelta*i+xoffset, j, halfcolor);
			else
				inputptr.SetPixel( xdelta*i+xoffset, j, color);
		}
		else if(chart_type==BAR_CHART)
		{
			inputptr.DrawLine( xdelta*i+xoffset, (_private::g_nChartPrecision-1)+yoffset, xdelta*i+xoffset, j, color);
		}

		prev_x=i;
		prev_y=j;
	}

	if(chart_type==LINE_CHART)
	{
		int prev_x=-1, prev_y;
		double* itr;

		for(int i=0; i<numFrame; i++)
		{
			itr=&aValue[i];

			int j=ToImageY(*itr, min, max, yoffset);
			inputptr.SetPixel( xdelta*i+xoffset, j, color);

			prev_x=i;
			prev_y=j;
		}
	}
}

#define C0 0
#define C1 128
#define C2 255

CPixelRGB8 GetColor(int i)
{
	switch(i%26)
	{
	case 0:	return CPixelRGB8(C0,C0,C1);
	case 1: return CPixelRGB8(C0,C0,C2);
	case 2: return CPixelRGB8(C0,C1,C0);
	case 3: return CPixelRGB8(C0,C1,C1);
	case 4: return CPixelRGB8(C0,C1,C2);
	case 5: return CPixelRGB8(C0,C2,C0);
	case 6: return CPixelRGB8(C0,C2,C1);
	case 7: return CPixelRGB8(C0,C2,C2);

	case 8: return CPixelRGB8(C1,C0,C0);
	case 9: return CPixelRGB8(C1,C0,C1);
	case 10: return CPixelRGB8(C1,C0,C2);
	case 11: return CPixelRGB8(C1,C1,C0);
	case 12: return CPixelRGB8(C1,C1,C1);
	case 13: return CPixelRGB8(C1,C1,C2);
	case 14: return CPixelRGB8(C1,C2,C0);
	case 15: return CPixelRGB8(C1,C2,C1);
	case 16: return CPixelRGB8(C1,C2,C2);

	case 17: return CPixelRGB8(C2,C0,C0);
	case 18: return CPixelRGB8(C2,C0,C1);
	case 19: return CPixelRGB8(C2,C0,C2);
	case 20: return CPixelRGB8(C2,C1,C0);
	case 21: return CPixelRGB8(C2,C1,C1);
	case 22: return CPixelRGB8(C2,C1,C2);
	case 23: return CPixelRGB8(C2,C2,C0);
	case 24: return CPixelRGB8(C2,C2,C1);
	case 25: return CPixelRGB8(C2,C2,C2);
	}
	return CPixelRGB8(0,0,0);
}

void resize(CImage& inout, int width, int height)
{
	CImage src;
	src.CopyFrom(inout);
	CImage* pInput=&src;
	CImagePixel cIP(pInput);
	CImage* pOutput=&inout;
	pOutput->Create(width, height);
	CImagePixel cOP(pOutput);
	float x,y;
	float x_ratio=(float)pInput->GetWidth()/(float)width;
	float y_ratio=(float)pInput->GetHeight()/(float)height;
	for(int i=0; i<width; i++)
	{
		for(int j=0; j<height; j++)
		{
			x=(float)i+0.5f;
			y=(float)j+0.5f;
			int count;
			cOP.SetPixel(i,j,cIP.GetPixel(x*x_ratio,y*y_ratio, count));
		}
	}
	/*
	static int hist=0;
	TString fn;
	fn.format("src%d.png", hist);
	pInput->Save(fn.ptr());
	fn.format("dest%d.png", hist);
	pOutput->Save(fn.ptr());
	hist++;
	*/
}















CImage* DrawChart(const matrixn& matrix, int chart_type, float min, float max , float horizLine)	//!< 0 frame부터 n-1프레임까지의 여러 signal들을 그린다.)
{
	if(min==max)
	{
		vectorn temp;
		temp.minimum(matrix);
		min=temp.minimum();
		temp.maximum(matrix);
		max=temp.maximum();
	}

	interval range(min, max);
	range.expand(0.0001);

	CImage* pImage=new CImage();
	pImage->Create(MAX(100, matrix.cols()), _private::g_nChartPrecision);

	CImagePixel cip(pImage);
	cip.Clear(CPixelRGB8(255,255,255));

	CPixelRGB8 color[3];
	color[0]=CPixelRGB8(200,0,0);
	color[1]=CPixelRGB8(0,200,0);
	color[2]=CPixelRGB8(0,0,200);

	for(int i=0; i<matrix.rows(); i++)
	{
		_private::DrawChart(pImage, matrix.cols(), matrix[i], range.start(), range.end(), color[i%3], 0, 0, 1,chart_type);
		if(horizLine!=FLT_MAX)
		{
			CImagePixel cip(pImage);
			int iy=ToImageY(horizLine, range.start(), range.end(), 0);
            cip.DrawHorizLine(0, iy, matrix.cols(), color[i%2+1]);
		}
	}

	return pImage;
}

CImage* DrawChart(const matrixn& matrix, float min, float max)
{
	CImage* pImage=new CImage();
	pImage->Create(matrix.rows(), _private::g_nChartPrecision);

	CImagePixel cip(pImage);
	cip.Clear(CPixelRGB8(255,255,255));

	CPixelRGB8 color[4];
	color[0]=CPixelRGB8(255,0,0);
	color[1]=CPixelRGB8(0,128,0);
	color[2]=CPixelRGB8(0,0,255);
	color[3]=CPixelRGB8(64,64,0);

	/* acceleration graph*/

	if(min==max)
	{
		min=matrix.toVector().minimum();
		max=matrix.toVector().maximum();
	}

	vectorn aColumn;
	for(int i=0; i<matrix.cols(); i++)
	{
		matrix.getColumn(i, aColumn);
		_private::DrawChart(pImage, matrix.rows(), aColumn.dataPtr(), min, max, color[i], 0, 0, 1, LINE_CHART);
	}

	return pImage;
}

CImage* DrawChart(const matrixn& matrix, int chart_type, vectorn const& _aMin, vectorn const& _aMax, double* aY)
{
	CImage* pImage=new CImage();
	pImage->Create(MAX(100, matrix.cols()), _private::g_nChartPrecision*matrix.rows());

	CImagePixel cip(pImage);
	cip.Clear(CPixelRGB8(255,255,255));

	CPixelRGB8 color[2];
	color[0]=CPixelRGB8(0,0,128);
	color[1]=CPixelRGB8(0,128,0);
	/* acceleration graph*/

	vectorn temp1, temp2;
	if(_aMin.size()==0)
		temp1.minimum(matrix);

	if(_aMax.size()==0)
		temp2.maximum(matrix);

	vectorn const& aMin=(_aMin.size())?_aMin:temp1;
	vectorn const& aMax=(_aMax.size())?_aMax:temp2;

	for(int i=0; i<matrix.rows(); i++)
	{
		_private::DrawChart(pImage, matrix.cols(), matrix[i], aMin[i], aMax[i], color[i%2], 0, _private::g_nChartPrecision*i, 1, chart_type);
		if(aY)
		{
			CImagePixel cip(pImage);
			int iy=ToImageY(aY[i], aMin[i], aMax[i], _private::g_nChartPrecision*i);
            cip.DrawHorizLine(0, iy, matrix.cols(), color[i%2+1]);
		}
	}

	return pImage;
}

CImage* DrawChart(const vectorn& vector, int chart_type, float min, float max)
{
	CImage* pImage=new CImage();
	pImage->Create(vector.size(), _private::g_nChartPrecision);

	CImagePixel cip(pImage);
	cip.Clear(CPixelRGB8(255,255,255));

	if(min==max)
	{
/*		intvectorn sorted;
		sorted.sortedOrder(vector);
		min=vector[sorted[sorted.size()/5]];
		max=vector[sorted[sorted.size()*4/5]];*/

		min=vector.minimum();
		max=vector.maximum();
	}

	interval range(min, max);
	range.expand(0.0001);

	_private::DrawChart(pImage, vector.size(), vector.dataPtr(), range.start(), range.end(), CPixelRGB8(0,0,0), 0, 0, 1,chart_type);

	return pImage;
}


CImage* DrawChart(const bitvectorn& ab, CPixelRGB8 color)
{
	int H=Imp::_private::g_nBoolChartPrecision;
	CImage* pImage=new CImage();
	pImage->Create(ab.size(), H);

	CImagePixel cip(pImage);
	cip.DrawBox(TRect(0,0,ab.size(), H), CPixelRGB8(0,0,0));
	for(int i=0; i<ab.size(); i++)
	{
		if(ab[i])
			cip.DrawVertLine(i, 0, H, color);
	}

	return pImage;
}

std::string defaultColormapFile()
{
	return RE::taesooLibPath()+"Resource/default/colormap.bmp";
}
CImage* DrawChart(const intvectorn& ab, const char* _colormapfile)
{
	std::string colormapfile;
	if(!_colormapfile)
		colormapfile=defaultColormapFile();
	else
		colormapfile=_colormapfile;
	
	CImage* pImage=new CImage();
	pImage->Create(ab.size(), 20);

	CImagePixel cip(pImage);

	CImage icolormap;
	icolormap.Load(colormapfile.c_str());
	CImagePixel colormap(&icolormap);

	int min=ab.minimum();
	int max=ab.maximum();

	intvectorn colormapIndex;
	colormapIndex.makeSamplingIndex2(icolormap.GetWidth(), max-min+1);

	for(int i=0; i<ab.size(); i++)
	{
		cip.DrawVertLine(i, 0, 20, colormap.GetPixel(colormapIndex[ab[i]-min],0));
	}

	return pImage;
}


CImage* DrawChartText(const ::intvectorn& ab, TStrings* translationTable)
{
	int upperBound=ab.maximum()+1;
	if(translationTable==NULL)
	{
		translationTable=new TStrings();
		translationTable->init(upperBound);
		for(int i=0; i<upperBound; i++)
		{
			translationTable->data(i).format("%d",i);
		}
	}
	else
	{
		ASSERT(translationTable->size()>=upperBound);
	}

	int maxLen=0;
	for(int i=0; i<upperBound; i++)
	{
		if(translationTable->data(i).length()>maxLen)
			maxLen=translationTable->data(i).length();
	}

#define FONT_HEIGHT 16
	CImage* pImage=new CImage();
	pImage->Create(ab.size(), FONT_HEIGHT*maxLen);

	CImagePixel cip(pImage);


	::intvectorn encoding;
	encoding.runLengthEncode(ab);

	int numGrp=encoding.size()/2;

	//OutputToFile("encoding.txt", TString("encoding %s\n", encoding.output().ptr()));

	for(int grp=0; grp<numGrp; grp++)
	{
		int start=encoding[grp*2];
		int end=encoding[(grp+1)*2];

		TString& text=translationTable->data(ab[start]);
		for(int i=0; i<text.length(); i++)
            cip.DrawText(start,FONT_HEIGHT*i, text.subString(i,i+1));
	}

	return pImage;
}
};

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

