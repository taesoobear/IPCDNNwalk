#include "stdafx.h"
#include "tvector.h"

vector2NView vector2N::range(int start, int end, int step)
{
	return _range<vector2NView >(start,end,step);
}

const vector2NView vector2N::range(int start, int end, int step) const	
{
	return ((const vector2N*)this)->range(start, end, step);
}
vector4    operator*( matrix4 const& mat, vector4 const& v) 
{
	m_real x=v[0];
	m_real y=v[1];
	m_real z=v[2];
	m_real w=v[3];
	
	m_real xx=mat._11*x+mat._12*y+mat._13*z+mat._14*w;
	m_real yy=mat._21*x+mat._22*y+mat._23*z+mat._24*w;
	m_real zz=mat._31*x+mat._32*y+mat._33*z+mat._34*w;
	m_real ww=mat._41*x+mat._42*y+mat._43*z+mat._44*w;
	return vector4(xx,yy,zz,ww);
}
