#ifndef _MATHCLASS_H_
#define _MATHCLASS_H_

#if _MSC_VER > 1000
#pragma once
//#pragma message("Compiling math_macro.h - this should happen just once per project.\n")
#endif

// 만약 renderer에서 사용할 것이면
#include "../stdafx.h"

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#include <stdio.h>
#include <stdarg.h>
//#include <tchar.h>

#include <math.h>
#include <assert.h>

#include <limits>

#ifndef	M_PI
#define	M_PI	3.14159265358979323846
#endif

#define EPS (1.0e-10)

#include <limits.h>
#include <float.h>
#include "math_macro.h"


// scalar binary operators
namespace s2
{
	m_real ADD(m_real a, m_real b) ;
	m_real SUB(m_real a, m_real b) ;
	m_real MULT(m_real a, m_real b);
	m_real DIV(m_real a, m_real b);
	m_real POW(m_real a, m_real b);
	m_real MINIMUM(m_real a, m_real b);
	m_real MAXIMUM(m_real a, m_real b);
	m_real GREATER(m_real a, m_real b);
	m_real GREATER_EQUAL(m_real a, m_real b);
	m_real SMALLER(m_real a, m_real b);
	m_real SMALLER_EQUAL(m_real a, m_real b);
	m_real EQUAL(m_real a, m_real b);
	m_real AVG(m_real a, m_real b);
	m_real BOUND(m_real a, m_real b);
	int INT_NOT_EQUAL(int a, int b);
	int INT_EQUAL(int a, int b);
}

#ifdef EXP 
#undef EXP
#endif
namespace s1
{
	// (scalar->scalar연산)
	void COS(m_real&b,m_real a);
	void SIN(m_real&b,m_real a);
	void EXP(m_real&b,m_real a);
	void NEG(m_real&b,m_real a);
	void SQRT(m_real&b,m_real a);
	void SQUARE(m_real&b,m_real a);
	void ASSIGN(m_real&b,m_real a);
	void LOG(m_real&b,m_real a);
	void abs(m_real&b,m_real a);
	void SMOOTH_TRANSITION(m_real&b,m_real a);
	void RADD(m_real&b,m_real a);
	void RDIV(m_real&b,m_real a);
	void RSUB(m_real&b,m_real a);
	void RMULT(m_real&b,m_real a);
	void BOUND(m_real&b, m_real a);
	void INVERSE(m_real&b, m_real a);
}


#include "quater.h"
#include "vector3.h"
#include "interval.h"
#include "matrix4.h"
#include "transf.h"
#include "traits.h"
#include "vectorn.h"
#include "bitVectorN.h"
#include "matrixn.h"
#endif
