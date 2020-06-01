#include "stdafx.h"
#include "mathclass.h"
#include "conversion.h"

vectornView vecView(matrixn const& a)
{
	int stride, n2, m2, on2;
	m_real* ptr;
	a._getPrivate(ptr, stride, n2, m2, on2);
	Msg::verify(a.cols()==stride, "vecView error! (a.cols()!=stride)");
	return vectornView(ptr, n2*m2, 1);
}

matrixnView matView(vectorn const& a, int nColumn)
{
	Msg::verify(a.size()%nColumn==0, "vectorn A cannot be viewed as matrixn");
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	return matrixnView(ptr, a.size()/nColumn, nColumn, nColumn);
}

matrixnView matView(matrixn const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	return a.range(start, end);
}

matrixnView matViewCol(matrixn const& a, int start, int end)
{
	if(end>a.cols()) end=a.cols();
	return a.range(0, a.rows(), start, end);
}

intmatrixnView intmatView(intmatrixn const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	return a.range(start, end);
}

intmatrixnView intmatViewCol(intmatrixn const& a, int start, int end)
{
	if(end>a.cols()) end=a.cols();
	return a.range(0, a.rows(), start, end);
}

matrixnView matView(vectorn const& a, int start, int end)
{
	if(end>a.size()) end=a.size();
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	
	if(end-start==0)
		return matrixnView(NULL, end-start, 1, stride);
	return matrixnView((m_real*)&a(start), end-start, 1, stride);
}
matrixnView matView(quaterN const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	
	if(end-start==0)
		return matrixnView(NULL, end-start, 4, stride);
	return matrixnView((m_real*)&a.row(start), end-start, 4, stride);
}

matrixnView matView(vector3N const& a)
{
	return matView(a,0);
}

matrixnView matView(vector3N const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	
	if(end-start==0)
		return matrixnView(NULL, end-start, 3, stride);
	return matrixnView((m_real*)&a.row(start), end-start, 3, stride);
}

quaterNView quatViewCol(matrixn const& a, int start)
{
	return a.range(0, a.rows(), start, start+4).toQuaterN();
}

vector3NView vec3ViewCol(matrixn const& a, int start)
{
	return a.range(0, a.rows(), start, start+3).toVector3N();
}


template <class T, class OutType>
OutType tvecViewOffset(_tvectorn<T, m_real> const& a, int start)
{
	m_real *ptr;
	int stride, n, on;
	a._getPrivate(ptr, stride, n, on);

	ptr=((m_real*)(ptr-start*stride));
	
	return OutType(ptr, n+start, stride);
}

vectornView vecViewOffset(vectorn const& a, int start)
{
	return tvecViewOffset<m_real, vectornView>(a,start);
	/*
	m_real *ptr;
	int stride, n, on;
	a._getPrivate(ptr, stride, n, on);

	ptr=((m_real*)(ptr-start*stride));
	return vectornView(ptr, n+start, stride);*/
}

vector3NView vec3ViewOffset(vector3N const& a, int start)
{
	return tvecViewOffset<vector3, vector3NView>(a,start);
}
quaterNView quatViewOffset(quaterN const& a, int start)
{
	return tvecViewOffset<quater, quaterNView>(a,start);
}


quaterNView quatView(vectorn const& v)
{
	m_real* ptr;
	int stride, n, on;
	v._getPrivate(ptr, stride, n, on);
	Msg::verify(stride==1 && n%4==0, "cannot be view as quaterN");
	return quaterNView (ptr, n/4, 4);

} 
vector3NView vec3View(vectorn const& v)
{
	m_real* ptr;
	int stride, n, on;
	v._getPrivate(ptr, stride, n, on);
	Msg::verify(stride==1 && n%3==0, "cannot be view as vector3N");
	return vector3NView (ptr, n/3, 3);
}
