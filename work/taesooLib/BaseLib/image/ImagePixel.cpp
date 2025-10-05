// ImagePixel.cpp: implementation of the CImagePixel class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ImagePixel.h"
#include "Image.h"

CImagePixel::CImagePixel()
{
	m_pInput=NULL;	
}

CImagePixel::CImagePixel(CImage* pInput)
{
	m_pInput=pInput;
	Init(pInput);
}

CImagePixel::~CImagePixel()
{
}

void CImagePixel::Init(CImage* pInput)
{
	m_pInput=pInput;

	m_pCPP.resize(m_pInput->GetHeight());

	for(int i=0; i<m_pInput->GetHeight(); i++)
	{
		m_pCPP[i]=m_pInput->GetPixel(0, i);
	}
}

CPixelRGB8 CImagePixel::GetPixel(float x, float y, int& count)
{
	///< bilinear filtering getpixel
	CPixelRGB8 color;
	int x1,y1,x2,y2;
	x1=(int)x;
	x2=x1+1;
	y1=(int)y;
	y2=y1+1;

	count=1;
	// 4개 픽셀중 하나라도 밖에 있는 경우 
	if(x1<0 || x2>=m_pInput->GetWidth() || y1<0 || y2>=m_pInput->GetHeight())
	{
		count=0;
		color=CPixelRGB8 (0,0,0);
	}
	// 모두 안에 있는경우 
	else
	{
		CPixelRGB8 c1,c2,c3,c4;

		float errx=x-x1;
		float erry=y-y1;
		float ex1=(1.f-errx)*(1.f-errx);
		float ex2=errx*errx;
		float ey1=(1.f-erry)*(1.f-erry);
		float ey2=erry*erry;
		
		// p1: x1,y1
		// p2: x1,y2
		// p3: x2,y1
		// p4: y2,y2
		float w1,w2,w3,w4;
		w1=ex1+ey1;
		w2=ex1+ey2;
		w3=ex2+ey1;
		w4=ex2+ey2;
		float sum=w1+w2+w3+w4;
		w1/=sum;
		w2/=sum;
		w3/=sum;
		w4/=sum;

		count=4;
		c1=GetPixel(x1,y1);
		c2=GetPixel(x1,y2);
		c3=GetPixel(x2,y1);
		c4=GetPixel(x2,y2);
		color=CPixelRGB8 (int((c1.R)*w1+(c2.R)*w2+(c3.R)*w3+(c4.R)*w4),
				  int((c1.G)*w1+(c2.G)*w2+(c3.G)*w3+(c4.G)*w4),
				  int((c1.B)*w1+(c2.B)*w2+(c3.B)*w3+(c4.B)*w4));
	}
	return color;
}
void CImagePixel::SetPixel(float fx, float fy, CPixelRGB8 color)
{
	int width=m_pInput->GetWidth();
	int height=m_pInput->GetHeight();
	
	int x,y;
	x=int(fx*width);
	y=int(fy*height);

	if(x<0) x=0;
	if(y<0) y=0;
	if(x>=width) x=width-1;
	if(y>=height) y=height-1;

	SetPixel(x,y,color);
}

void CImagePixel::DrawHorizLine(int x, int y, int width, CPixelRGB8 color)
{
	{
		std::vector<CPixelRGB8 *> & inputptr=m_pCPP;
	
		if(x<0) return;
		if(x>=m_pInput->GetWidth()) return;
		if(y<0) return;
		if(y>=m_pInput->GetHeight()) return;
		if (x+width>=m_pInput->GetWidth())
			width=m_pInput->GetWidth()-x-1;

		for(int i=x; i<x+width; i++)
		{
			inputptr[y][i]=color;
		}
	}
}

void CImagePixel::DrawVertLine(int x, int y, int height, CPixelRGB8 color,bool bDotted)
{
	int step=1;
	if(bDotted) step=3;
	std::vector<CPixelRGB8 *> & inputptr=(m_pCPP);

		if(x<0) return;
		if(x>=m_pInput->GetWidth()) return;
		if(y<0) return;
		if(y>=m_pInput->GetHeight()) return;
		if (y+height>=m_pInput->GetHeight())
			height=m_pInput->GetHeight()-y-1;

	for(int j=y; j<y+height; j+=step)
	{
		inputptr[j][x]=color;
	}
}


void CImagePixel::DrawLineBox(const TRect& rect, CPixelRGB8 color)
{
	DrawHorizLine(rect.left, rect.top, rect.Width(), color);
	DrawHorizLine(rect.left, rect.bottom-1, rect.Width(), color);
	DrawVertLine(rect.left, rect.top, rect.Height(), color);
	DrawVertLine(rect.right-1, rect.top, rect.Height(), color);
}
void CImagePixel::DrawBox(const TRect& _rect, CPixelRGB8 sColor)
{
	TRect rect=_rect;
	if(rect.left> rect.right) std::swap(rect.left, rect.right);
	if(rect.top> rect.bottom) std::swap(rect.top, rect.bottom);
	if(rect.left<0) rect.left=0;
	if(rect.top<0) rect.top=0;
	if(rect.bottom>Height())rect.bottom=Height();
	if(rect.right>Width())rect.right=Width();

	{

		std::vector<CPixelRGB8 *> & inputptr=m_pCPP;

		/*
		// easy to read version
		for(int j=rect.top; j<rect.bottom; j++)
		{
			CPixelRGB8* ptr=inputptr[j];
			for(int i=rect.left; i<rect.right; i++)
			{
				memcpy(&ptr[i],&sColor,sizeof(CPixelRGB8));
			}
		}
		*/
		// fast version
		CPixelRGB8* aBuffer;
		int width=rect.right-rect.left;
		if(width>0)
		{
			aBuffer=new CPixelRGB8[width];
			for(int i=0; i<width; i++)
				aBuffer[i]=sColor;
			for(int j=rect.top; j<rect.bottom; j++)
			{
				CPixelRGB8* ptr=inputptr[j];

				memcpy(&ptr[rect.left],aBuffer, sizeof(CPixelRGB8)*(width));
			}
			delete[] aBuffer;
		}
	}	
}

void CImagePixel::Clear(CPixelRGB8 color)
{
	int width=m_pInput->GetWidth();
	int height=m_pInput->GetHeight();

	DrawBox(TRect(0,0, width, height), color);
}

void CImagePixel::DrawPattern(int x, int y, const CImagePixel& patternPixel, bool bUseColorKey, CPixelRGB8 sColorkey, bool bOverideColor, CPixelRGB8 overrideColor)
{
	int imageWidth=m_pInput->GetWidth();
	int imageHeight=m_pInput->GetHeight();
	int patternWidth=patternPixel.m_pInput->GetWidth();
	int patternHeight=patternPixel.m_pInput->GetHeight();

	int imagex, imagey;
	
	if(bUseColorKey)
	{
		if(bOverideColor)
		{
			float ovR=float((overrideColor.R))/255.f;
			float ovG=float((overrideColor.G))/255.f;
			float ovB=float((overrideColor.B))/255.f;
			for(int j=0; j<patternHeight; j++)
				for(int i=0; i<patternWidth; i++)
				{
					imagex=x+i; imagey=y+j;
					if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
					{
						if(memcmp(&patternPixel.GetPixel(i,j),&sColorkey, sizeof(CPixelRGB8))!=0)
						{
						//	SetPixel( imagex, imagey, overrideColor);
							CPixelRGB8& c=Pixel(imagex, imagey);
							CPixelRGB8& cc=patternPixel.Pixel(i,j);
							c.R=cc.R*ovR;
							c.G=cc.G*ovG;
							c.B=cc.B*ovB;							
						}
					}
				}
		}
		else
		{
			for(int j=0; j<patternHeight; j++)
				for(int i=0; i<patternWidth; i++)			
				{
					imagex=x+i; imagey=y+j;
					if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
					{
						if(memcmp(&patternPixel.Pixel(i,j),&sColorkey, sizeof(CPixelRGB8))!=0)
							GetPixel(imagex,imagey)=patternPixel.GetPixel(i,j);
					}
				}
		}
	}
	else
	{
		ASSERT(!bOverideColor);
		for(int j=0; j<patternHeight; j++)
		{
			CPixelRGB8* target=&GetPixel(x,y+j);
			CPixelRGB8* source=&patternPixel.Pixel(0,j);
			memcpy(target,source, sizeof(CPixelRGB8)*patternWidth);
		}
	}
}

void CImagePixel::DrawPattern(int x, int y, CImage* pPattern, bool bUseColorkey, CPixelRGB8 colorkey, bool bOverideColor, CPixelRGB8 overrideColor)
{
	CImagePixel patternPixel(pPattern);
	DrawPattern(x,y,patternPixel,bUseColorkey,colorkey,bOverideColor,overrideColor);
}

void CImagePixel::DrawLine(int x1, int y1, int x2, int y2, CPixelRGB8 color)	//!< inputptr, inputptr2중 하나는 NULL로 줄것.
{
	int dx,dy,x,y,x_end,p,const1,const2,y_end;
	int delta;
	dx=abs(x1-x2);
	dy=abs(y1-y2);
	
	if (((y1-y2)>0 && (x1-x2)>0 ) || (y1-y2)<0 && (x1-x2)<0)
	{
		delta=1;							//기울기 >0
	}
	else
	{
		delta=-1;							//기울기 <0
	}
	
	if(dx>dy)								//기울기 0 < |m| <=1
	{
		p=2*dy-dx;
		const1=2*dy;
		const2=2*(dy-dx);
		if(x1>x2)
		{
			x=x2;y=y2;
			x_end=x1;
		}
		else
		{
			x=x1;y=y1;
			x_end=x2;
		}
	
		SetPixel( x,y, color);
		while(x<x_end)
		{
			x=x+1;
			if(p<0)
			{
				p=p+const1;
			}
			else
			{
				y=y+delta;
				p=p+const2;
			}
			SetPixel( x,y, color);
		}									//기울기 |m| > 1
	}
	else
	{
		p=2*dx-dy;
		const1=2*dx;
		const2=2*(dx-dy);
		if(y1>y2)
		{
			y=y2;x=x2;
			y_end=y1;
		}
		else
		{
			y=y1;x=x1;
			y_end=y2;
		}
		
		SetPixel( x,y, color);
		while(y<y_end)
		{
			y=y+1;
			if(p<0)
			{
				p=p+const1;
			}
			else
			{
				x=x+delta;
				p=p+const2;
			}

			SetPixel( x,y, color);
		}
	}
}

void CImagePixel::DrawSubPattern(int x, int y, const CImagePixel& patternPixel, const TRect& patternRect, bool bUseColorKey, CPixelRGB8 sColorkey)
{
	int imageWidth=m_pInput->GetWidth();
	int imageHeight=m_pInput->GetHeight();
	int patternWidth=patternPixel.m_pInput->GetWidth();
	int patternHeight=patternPixel.m_pInput->GetHeight();

	ASSERT(patternRect.right<=patternWidth);
	ASSERT(patternRect.top<=patternHeight);

	int imagex, imagey;

	
	if(bUseColorKey)
	{
		for(int j=patternRect.top; j<patternRect.bottom; j++)
			for(int i=patternRect.left; i<patternRect.right; i++)			
			{
				imagex=x+i-patternRect.left; imagey=y+j-patternRect.top;
				if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
				{
					if(memcmp(&patternPixel.Pixel(i,j),&sColorkey, sizeof(CPixelRGB8))!=0)
						Pixel(imagex,imagey)=patternPixel.Pixel(i,j);
				}
			}
	}
	else
	{
		TRect rect=patternRect;
		if(x<0)
		{
			rect.left-=x;
			x-=x;
		}
		if(x+rect.Width()>imageWidth)
		{
			int delta=x+rect.Width()-imageWidth;
			rect.right-=delta;			
		}
		if(rect.Width()>0)
		{
			for(int j=rect.top; j<rect.bottom; j++)
			{
				imagey=y+j-rect.top;
				if(imagey>=0 && imagey <imageHeight)
				{
					CPixelRGB8* target=&Pixel(x,imagey);
					CPixelRGB8* source=&patternPixel.Pixel(rect.left,j);
					memcpy(target,source, sizeof(CPixelRGB8)*rect.Width());
				}
			}
		}
	}
}

void CImagePixel::DrawText(int x, int y, const char* str, bool bUserColorKey, CPixelRGB8 colorkey)
{
	static CImage* pText=NULL;
	if(!pText)
	{
		pText=new CImage();
		pText->Load("../Resource/default/ascii.bmp");
	}
	CImage& cText=*pText;
    
	CImagePixel patternPixel(&cText);
#define FONT_HEIGHT 16
#define FONT_WIDTH 8
	int len=strlen(str);
	for(int i=0; i<len; i++)
	{
		char c=str[i];
		int code=(c-' ');
		ASSERT(code>=0 && code<32*3);
		int left=code%32*FONT_WIDTH ;
		int top=(2-code/32)*FONT_HEIGHT;
		DrawSubPattern(x+i*FONT_WIDTH , y, patternPixel, TRect(left,top,left+FONT_WIDTH , top+FONT_HEIGHT), bUserColorKey, colorkey);
	}
}

CEditableImage ::CEditableImage (int width, int height)
	{
		m_pImage=new CImage();
		m_pImage->Create(width, height);
		Init(m_pImage);
	}

CEditableImage ::	~CEditableImage ()
	{
		delete m_pImage;
	}
