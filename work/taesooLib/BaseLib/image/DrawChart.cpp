#include "stdafx.h"
#include "../math/mathclass.h"
#include "DrawChart.h"
#include "../utility/operatorString.h"

#define SIZE 780
#define AXISX 80
#define AXISY 700
#define CANVAS_SIZE 650

int DrawChart::x(m_real fx)
{
	int x;
	x=int(fx*SIZE);

	if(x<0) x=0;
	if(x>=SIZE) x=SIZE-1;
	return x;
}

int DrawChart::y(m_real fy)
{
	int y=int(fy*SIZE);
	if(y<0) y=0;
	if(y>=SIZE) y=SIZE-1;
	return y;
}

m_real DrawChart::paramX(int x)	// inverse of canvasX
{
	//x=int(alpha*CANVAS_SIZE)+AXISX+1;
	m_real alpha=(m_real(AXISX+1-x))/CANVAS_SIZE;
	return alpha;
}

m_real DrawChart::paramY(int y)	// inverse of canvasY
{
	//y=AXISY-int(alpha*CANVAS_SIZE)-1;
	m_real alpha=(m_real(AXISY-1-y))/CANVAS_SIZE;
	return alpha;
}

int DrawChart::canvasX(m_real x)
{
	m_real alpha=mInterval[0].uninterpolate(x);
	return int(alpha*CANVAS_SIZE)+AXISX+1;
}

int DrawChart::canvasY(m_real y)
{
	m_real alpha=mInterval[1].uninterpolate(y);
	return AXISY-int(alpha*CANVAS_SIZE)-1;
}


DrawChart::DrawChart(const char* xaxis, const char* yaxis, m_real minx, m_real maxx, m_real miny, m_real maxy)
{
	mXaxis=xaxis;
	mYaxis=yaxis;
	mInterval.setSize(2);
	mInterval[0].setValue(minx,maxx);
	mInterval[1].setValue(miny,maxy);
	mpImage=new CImage();
	mpImage->Create(SIZE, SIZE);
	mpCanvas.Init(mpImage);
	mpCanvas.Clear(CPixelRGB8(255,255,255));
}

DrawChart::~DrawChart(void)
{
	delete mpImage;
}

void DrawChart::drawMatrix(matrixn const& matrix)
{
	vectorn xx, yy;
	xx.linspace(mInterval[0].start(), mInterval[0].end(), matrix.cols());
	yy.linspace(mInterval[1].start(), mInterval[1].end(), matrix.rows());

	vectorn cx, cy;
	cx.setSize(matrix.cols());
	cy.setSize(matrix.rows());

	for(int i=0; i<matrix.cols(); i++)
		cx[i]=canvasX(xx[i]);

	for(int i=0; i<matrix.rows(); i++)
		cy[i]=canvasY(yy[i]);

	int width=cx[1]-cx[0]+1;
	int height=-1.0*(cy[1]-cy[0])+1;

	width/=2; height/=2;
	width+=1;
	height+=1;
	m_real max=matrix.maximum();
	m_real min=matrix.minimum();

	mpCanvas.DrawText(canvasX(mInterval[0].start()), AXISY+20, sz1::format("min, max %f %f", min, max));


	if(min==max)
	{
		min-=0.1;
		max+=0.1;
	}	


	for(int i=0; i<matrix.rows(); i++)
		for(int j=0; j<matrix.cols(); j++)
		{
			int color=(int)((matrix[i][j]-min)*255.f/(max-min));
			//printf("color %d %d %f %d %f %d %d\n", width, height, xx[i], (int)cx[i], yy[j], (int)cy[j], color);
			mpCanvas.DrawBox(TRect(cx[i]-width, cy[j]-height, cx[i]+width, cy[j]+height), CPixelRGB8(color, color, color));
		}

}


void DrawChart::drawScatteredData(matrixn const& matrix, CPixelRGB8 color, char* patternFile)
{
	CImage pattern;
	int dist;
	if(patternFile)
	{	
		pattern.Load(patternFile);
		dist=pattern.GetWidth()/2;
	}

	if(patternFile)
	{
		if(color==CPixelRGB8(255,255,255))
			for(int i=0; i<matrix.rows(); i++)
			{
				int x=canvasX(matrix[i][0]);
				int y=canvasY(matrix[i][1]);
				mpCanvas.DrawPattern(x-dist, y-dist, &pattern, true, CPixelRGB8(0,0,0));
			}
		else
			for(int i=0; i<matrix.rows(); i++)
			{
				int x=canvasX(matrix[i][0]);
				int y=canvasY(matrix[i][1]);
				mpCanvas.DrawPattern(x-dist, y-dist, &pattern, true, CPixelRGB8(0,0,0), true, color);
			}
	}
	else
	{
		for(int i=0; i<matrix.rows(); i++)
		{
			int x=canvasX(matrix[i][0]);
			int y=canvasY(matrix[i][1]);
			mpCanvas.SetPixel( x, y ,color);
			mpCanvas.SetPixel( x, y-1 ,color);
			mpCanvas.SetPixel( x, y+1 ,color);
			mpCanvas.SetPixel( x, y-2 ,color);
			mpCanvas.SetPixel( x, y+2 ,color);

			mpCanvas.SetPixel( x-1, y ,color);
			mpCanvas.SetPixel( x+1, y ,color);
			mpCanvas.SetPixel( x-2, y ,color);
			mpCanvas.SetPixel( x+2, y ,color);
		}
	}
	
}

void DrawChart::save(const char* filename)
{
	mpCanvas.DrawText(0, canvasY(mInterval[1].start()), sz1::format("%f", mInterval[1].start()));
	mpCanvas.DrawText(0, canvasY(mInterval[1].end()), sz1::format("%f", mInterval[1].end()));
	mpCanvas.DrawText( canvasX(mInterval[0].start()), AXISY+10, sz1::format("%f", mInterval[0].start()));
	mpCanvas.DrawText( canvasX(mInterval[0].end()), AXISY+10, sz1::format("%f", mInterval[0].end()));

	mpImage->Save(filename);
}
