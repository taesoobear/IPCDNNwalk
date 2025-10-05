#ifndef _CONVERSION_H_
#define _CONVERSION_H_

#if _MSC_VER>1000
#pragma once
#endif

// from matrix
matrixnView matView(matrixn const& a, int start=0, int end=INT_MAX);
matrixnView matViewCol(matrixn const& a, int startCol=0, int endCol=INT_MAX);
intmatrixnView intmatView(intmatrixn const& a, int startRow=0, int endRow=INT_MAX);
intmatrixnView intmatViewCol(intmatrixn const& a, int startCol=0, int endCol=INT_MAX);
quaterNView quatViewCol(matrixn const& a, int startCol=0);
quaterNView quatView(vectorn const& a);

vector3NView vec3ViewCol(matrixn const& a, int startCol=0);
vector3NView vec3View(vectorn const& a);

vectornView vecView(matrixn const& a);

// from quaterN
matrixnView matView(quaterN const& a, int start=0, int end=INT_MAX);

// from vector3N
matrixnView matView(vector3N const& a, int start, int end=INT_MAX);
matrixnView matView(vector3N const& a);

// from vectorn
matrixnView matView(vectorn const& a, int nColumn);
matrixnView matView(vectorn const& a, int start, int end);

// b=vecViewOffset(a,3); --> b[3]==a[0]. 주의사항: 범위 에러 체크는 하지 않음.
vectornView vecViewOffset(vectorn const& a, int start);
vector3NView vec3ViewOffset(vector3N const& a, int start);
quaterNView quatViewOffset(quaterN const& a, int start);

template <class T>
vectornView vecView(_tvectorn<T, m_real> const& v)
{
	m_real* ptr;
	int stride, n, on;
	v._getPrivate(ptr, stride, n, on);
	int T_N=sizeof(T)/sizeof(m_real);
	Msg::verify(stride==T_N, "cannot be viewed as vectorn");
	return vectornView(ptr, n*T_N, 1);
}

#endif
