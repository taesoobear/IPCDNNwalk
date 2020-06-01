#ifndef _MATH_MACRO_H_
#define _MATH_MACRO_H_
#if _MSC_VER > 1000
#pragma once
//#pragma message("Compiling math_macro.h - this should happen just once per project.\n")
#endif

#define TRUE    1
#define FALSE   0

typedef double m_real;

#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#define ABS(x)   ( ((x)>0.0) ? (x) :(-1.0*(x)) )
#define ACOS(x)  ( ((x)>1.0) ? (0) : ( ((x)<-1.0) ? (M_PI) : (acos(x)) ) )
#define ASIN(x)  ( ((x)>1.0) ? (M_PI/2.0) : ( ((x)<-1.0) ? (-M_PI/2.0) : (asin(x)) ) )
#define SQR(x)   ( (x)*(x) )
#define SHIFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define ROUND(x) ((int)floor((x)+0.5))
#define TO_RADIAN(degree) ((degree) * (M_PI / 180.0f))
#define TO_DEGREE(radian) ((radian)/(M_PI/180.f))

template<class T>
inline void SWAP(T &a, T &b)
{T dum=a; a=b; b=dum;}

template <class T>
inline const T CUBIC(const T x){return x*x*x;}


inline m_real MAX3(m_real a,m_real b, m_real c) {
	if(a>b)
	{
		if(a>c) return a;
		else return c;
	}
	else
	{
		if(c>b) return c;
		else return b;
	}
}

inline m_real MIN3(m_real a,m_real b, m_real c) {
	if(a<b)
	{
		if(a<c) return a;
		else return c;
	}
	else
	{
		if(c<b) return c;
		else return b;
	}
}

inline m_real CLAMP(m_real a, m_real i1, m_real i2)
{
	if(a<i1) return i1;
	if(a>i2) return i2;
	return a;
}

inline int CLAMP(int a, int i1, int i2)
{
	if(a<i1) return i1;
	if(a>i2) return i2;
	return a;
}

inline bool isSimilar(m_real a, m_real b)
{
	return ABS(a-b)<0.0001;
}

inline bool isSimilar(m_real a, m_real b, m_real thr)
{
	return ABS(a-b)<thr;
}


class CAggregate
{
public:
	enum aggregateOP { LENGTH, RMS, SUM, AVG, SQUARESUM, MINIMUM, MAXIMUM } ;
	CAggregate(aggregateOP op)
	{
		switch(op)
		{
		case LENGTH:
		case RMS:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSquareSum;
			final_function=&CAggregate::FinalSqrt;
			break;
		case SUM:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSum;
			final_function=&CAggregate::FinalCur;
			break;
		case AVG:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSum;
			final_function=&CAggregate::FinalDivN;
			break;
		case SQUARESUM:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSquareSum;
			final_function=&CAggregate::FinalCur;
			break;
		case MINIMUM:
			init_function=&CAggregate::InitFMax;
			update_function=&CAggregate::UpdateMin;
			final_function=&CAggregate::FinalCur;
			break;
		case MAXIMUM:
			init_function=&CAggregate::InitFMin;
			update_function=&CAggregate::UpdateMax;
			final_function=&CAggregate::FinalCur;
			break;
		}
	};
	~CAggregate(){};

	inline m_real Init() { return (this->*init_function)();};
	inline void Update(m_real& cur, m_real v) { (this->*update_function)(cur,v);};
	inline m_real Final(m_real v, int n) { return (this->*final_function)(v,n);};
private:
	m_real (CAggregate::*init_function)() const;
	void (CAggregate::*update_function)(m_real &cur, m_real v) const;
	m_real (CAggregate::*final_function)(m_real cur, int n) const;

	m_real InitZero() const	;
	m_real InitFMax() const	;
	m_real InitFMin() const	;
	void UpdateSquareSum(m_real &cur, m_real v) const;
	void UpdateMin(m_real &cur, m_real v) const;
	void UpdateMax(m_real &cur, m_real v) const;
	void UpdateSum(m_real &cur, m_real v) const;
	m_real FinalSqrt(m_real cur, int n) const	;
	m_real FinalCur(m_real cur, int n) const	;
	m_real FinalDivN(m_real cur, int n) const	;
};
#endif
