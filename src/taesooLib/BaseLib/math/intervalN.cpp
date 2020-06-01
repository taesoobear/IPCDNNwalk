#include "stdafx.h"
#include "mathclass.h"
#include "intervalN.h"

intervalN::intervalN( vectorn const &a, vectorn const &b )
{
	m_vStart.each2(s2::MINIMUM, a,b);
	m_vEnd.each2(s2::MAXIMUM, a,b);
}

/*
ostream& operator<<( ostream& os, intervalN const& a )
{
    os << "[ " << a.m_vStart << " , " << a.m_vEnd << " ]";
    return os;
}

istream& operator>>( istream& is, intervalN& a )
{
	static char	buf[256];
    //is >> "[" >> a.m_vStart >> "," >> a.m_vEnd >> "]";
	is >> buf >> a.m_vStart >> buf >> a.m_vEnd >> buf;
    return is;
}
*/

intervalN operator-( intervalN const& a )
{
    return intervalN( -a.m_vEnd, -a.m_vStart);
}

intervalN& operator+=( intervalN& a, vectorn const& b )
{
    a.start_ref() += b;
    a.end_ref()   += b;

    return a;
}

intervalN operator+( intervalN const& a, vectorn const& b )
{
    return intervalN( a.start() + b, a.end() + b );
}

intervalN operator+( vectorn const& b, intervalN const& a )
{
    return intervalN( a.start() + b, a.end() + b );
}

intervalN& operator-=( intervalN& a, vectorn const& b )
{
    a.start_ref() -= b;
    a.end_ref()   -= b;

    return a;
}

intervalN operator-( intervalN const& a, vectorn const& b )
{
    return intervalN( a.start() - b, a.end() - b );
}

intervalN& operator*=( intervalN & a, m_real b )
{
    a.start_ref() *= b;
    a.end_ref()   *= b;
    adjust_intervalN( a );

    return a;
}

intervalN operator*( intervalN const& a, m_real b )
{
    intervalN c( a.start() * b, a.end() * b );
    adjust_intervalN( c );

    return c;
}

intervalN operator*( m_real b, intervalN const& a )
{
    intervalN c( a.m_vStart * b, a.m_vEnd * b );
    adjust_intervalN( c );

    return c;
}

intervalN& operator/=( intervalN& a, m_real b )
{
    a.m_vStart /= b;
    a.m_vEnd   /= b;
    adjust_intervalN( a );

    return a;
}

intervalN operator/( intervalN const& a, m_real b )
{
    intervalN c( a.m_vStart / b, a.m_vEnd / b );
    adjust_intervalN( c );

    return c;
}

intervalN& operator|=( intervalN& a, intervalN const& b )
{
	a.m_vStart.each2(s2::MINIMUM, a.m_vStart, b.m_vStart );
	a.m_vEnd.each2(s2::MAXIMUM, a.m_vEnd  , b.m_vEnd   );
	return a;
}

intervalN operator|( intervalN const& a, intervalN const& b )
{
	vectorn c,d;
	c.each2(s2::MINIMUM, a.m_vStart, b.m_vStart );
	d.each2(s2::MAXIMUM, a.m_vEnd, b.m_vEnd );
	return intervalN(c , d );
}

int operator>>( intervalN const& a, vectorn const& b )
{
    if ( a.start()<b && b<a.end() ) return TRUE;
                                   else return FALSE;
}

int operator<<( vectorn const&b, intervalN const& a )
{
    if ( a.start()<b && b<a.end()) return TRUE;
                                   else return FALSE;
}

int operator>>( intervalN const& a, intervalN const& b )
{
    if ( a.start()<b.start()&& b.end()<a.end()) return TRUE;
                                             else return FALSE;
}

int operator<<( intervalN const& b, intervalN const& a )
{
    if ( a.start()<b.start() && b.end()<a.end() ) return TRUE;
                                             else return FALSE;
}

intervalN& operator&=( intervalN& a, intervalN const& b )
{
    if ( a<<b ) ;
    else if ( b<<a ) { a = b; }
    else if ( b.m_vEnd<a.m_vStart ) { a *= 0.0; }
    else if ( a.m_vEnd<b.m_vStart ) { a *= 0.0; }
    else if ( a.m_vStart<b.m_vStart ) { a.m_vStart = b.m_vStart; }
    else { a.m_vEnd = b.m_vEnd; }
                               
    return a;
}

intervalN operator&( intervalN const& a, intervalN const& b )
{
    intervalN c( a.start());

    if ( a<<b ) { c = a; }
    else if ( b<<a ) { c = b; }
    else if ( b.m_vEnd<a.m_vStart ) { c *= 0.0; }
    else if ( a.m_vEnd<b.m_vStart ) { c *= 0.0; }
    else if ( a.m_vStart<b.m_vStart ) { c.m_vStart = b.m_vStart; c.m_vEnd = a.m_vEnd; }
    else { c.m_vStart = a.m_vStart; c.m_vEnd = b.m_vEnd; }
                               
    return c;
}

int operator&&( intervalN const& a, intervalN const& b )
{
    if ( a.start()> b.end() || a.end() < b.start() ) return FALSE;
                                                 else return TRUE;
}

intervalN operator^( intervalN const& a, vectorn const& b )
{
    return intervalN( a.start() - b, a.end() + b );
}

void adjust_intervalN( intervalN& a )
{
    if ( a.m_vStart > a.m_vEnd )
    {
        vectorn& temp = a.m_vStart;
        a.m_vStart = a.m_vEnd;
        a.m_vEnd = temp;
    }
}

void intervalN::project( vectorn const& d, vectorn& out, vectorn & ProjectAxis ) const
{
	ProjectAxis.setSize(d.size());
	ProjectAxis.setAllValue(0);

	out=d;
	for(int i=0; i<d.size(); i++)
	{
		if ( start(i) > d[i] )
		{
			out[i]=start(i);
			ProjectAxis[i]=1;
		}
		else if ( end(i) < d[i] )
		{
			out[i]=end(i);
			ProjectAxis[i]=-1;
		}
	}
}
/*
vectorn intervalN::project( vectorn const& d, vectorn & ProjectAxis ) const
{
	ProjectAxis.setSize(d.size());
	ProjectAxis.setAllValue(0);

	vectorn out=d;
	for(int i=0; i<d.size(); i++)
	{
		if ( start(i) > d[i] )
		{
			out[i]=start(i);
			ProjectAxis[i]=1;
		}
		else if ( end(i) < d[i] )
		{
			out[i]=end(i);
			ProjectAxis[i]=-1;
		}
	}
	return out;
}
*/

m_real
intervalN::distance( vectorn const& d ) const
{
	ASSERT(0);
    return 0;
}

void intervalN::mid_pt(vectorn& midpoint)
{
	midpoint=(m_vStart+m_vEnd)/2.0f;
}

void intervalN::interpolate( m_real t,vectorn& result ) const
{
	result=m_vStart*(1-t) + m_vEnd*t; 
}

void intervalN::interpolate( const vectorn& t,vectorn& result ) const
{
	// component wise
	vectorn one(t.size());
	one.setAllValue(1);
	
	result.mult( m_vStart,(one-t));
	vectorn temp;
	temp.mult( m_vEnd,t); 

	result+=temp;
}

void intervalN::uninterpolate( const vectorn& v, vectorn& nv) const
{
	ASSERT(v.size()==size());
	nv.setSize(v.size());
	for(int i=0; i<size(); i++)
	{
		nv[i]=getInterval(i).uninterpolate(v[i]);
	}
	//!< normalize input using this interval.
}

void intervalN::getRange(vectorn & range) const
{
	range.sub(m_vEnd,m_vStart);
}

void intervalN::calcRange(const matrixn& input)
{
	m_vStart.minimum(input);
	m_vEnd.maximum(input);
}

void intervalN::normalizeRange(const vectorn& aspectRatio)
{
	vectorn delta;
	delta.sub(end(), start());

	for(int i=0; i<delta.size(); i++)
	{
		delta[i]/=aspectRatio[i];
	}

	int argMax=delta.argMax();

	float maxPerRatio=delta[argMax];

	for(int i=0; i<delta.size(); i++)
	{
		m_real center=start()[i]+end()[i];
		center/=2.f;

		m_real halfrange=maxPerRatio*aspectRatio[i]/2.f;
		start()[i]=center-halfrange;
		end()[i]=center+halfrange;
	}
}

void intervalN::calcOutlierFence(const matrixn& input, m_real safety)
{
	//!< find range excluding mild outliers
	m_vStart.setSize(input.cols());
	m_vEnd.setSize(input.cols());

	vectorn colVec;
	interval itv;
	for(int i=0; i<input.cols(); i++)
	{
		input.getColumn(i, colVec);
		itv.calcOutlierFence(colVec, safety);
		m_vStart[i]=itv.start_pt();
		m_vEnd[i]=itv.end_pt();
	}
}

void intervalN::initEmpty()
{
	m_vStart.setAllValue(FLT_MAX);
	m_vEnd.setAllValue(-FLT_MAX);
}

void intervalN::enlarge(const vectorn& point)
{
	//!< enlarge interval so that it include the point
	m_vStart.each2(s2::MINIMUM, m_vStart, point);
	m_vEnd.each2(s2::MAXIMUM, m_vEnd, point);
}

void intervalN::expand(const vectorn& epsilon)	//!< min=min-epsilon, max=max+epsilon
{
	m_vStart-=epsilon;
	m_vEnd+=epsilon;
}

void intervalN::rand_pt(vectorn& randpoint)
{
	// sampling using uniform distribution.
	randpoint.setSize(size());
	for(int i=0; i<size(); i++)
		randpoint[i]=getInterval(i).rand_pt();
}

void intervalN::assign(const vectorn& a, const vectorn& b)
{
	m_vStart.each2(s2::MINIMUM,a,b);
	m_vEnd.each2(s2::MAXIMUM,a,b);
}
