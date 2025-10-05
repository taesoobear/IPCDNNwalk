
#include "stdafx.h"
#include "NumericalSpline.h"


void CubicPolynomial::getCurve(const vectorn &time, matrixn &points)
{
	m_real a, b, c, d, t;
	int dim=mSolvers.size();
	points.setSize(time.size(),dim);

	for(int j=0; j<dim; j++)
	{
		for(int i=0; i<time.size(); i++)
		{
			a=coef[j][0];
			b=coef[j][1];
			c=coef[j][2];
			d=coef[j][3];			
			t=time[i];
			points[i][j]=a*CUBIC(t)+b*SQR(t)+c*t+d;
		}		
	}	
}

void CubicPolynomial::getFirstDeriv(const vectorn &time, matrixn &points)
{
	m_real a, b, c, d, t;
	int dim=mSolvers.size();
	points.setSize(time.size(),dim);

	for(int j=0; j<dim; j++)
	{
		for(int i=0; i<time.size(); i++)
		{
			a=coef[j][0];
			b=coef[j][1];
			c=coef[j][2];
			d=coef[j][3];			
			t=time[i];
			points[i][j]=3*a*SQR(t)+2*b*t+c;
		}		
	}	
}

void CubicPolynomial::getSecondDeriv(const vectorn &time, matrixn& points)
{
	m_real a, b, c, d, t;
	int dim=mSolvers.size();
	points.setSize(time.size(),dim);

	for(int j=0; j<dim; j++)
	{
		for(int i=0; i<time.size(); i++)
		{
			a=coef[j][0];
			b=coef[j][1];
			c=coef[j][2];
			d=coef[j][3];			
			t=time[i];
			points[i][j]=6*a*t+2*b;
		}		
	}	
}
