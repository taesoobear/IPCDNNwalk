#include "stdafx.h"
#include "mathclass.h"

void interval::enlarge(m_real point)
{
	//!< enlarge interval so that it include the point
	m_start=MIN(m_start, point);
	m_end=MAX(m_end, point);
}

void interval::expand(m_real epsilon)	//!< min=min-epsilon, max=max+epsilon
{
	m_start-=epsilon;
	m_end+=epsilon;
}

bool interval::isInside(m_real value) const
{
	if(value<m_start || value>m_end)
		return false;
	return true;
}

std::ostream& operator<<( std::ostream& os, interval const& a )
{
    os << "[ " << a.m_start << " , " << a.end_pt() << " ]";
    return os;
}

std::istream& operator>>( std::istream& is, interval& a )
{
	static char	buf[256];
	is >> a.m_start >> a.end() ;
    return is;
}
interval operator-( interval const& a )
{
    return interval( -a.m_end, -a.m_start );
}

interval& operator+=( interval& a, m_real b )
{
    a.m_start += b;
    a.m_end   += b;

    return a;
}

interval operator+( interval const& a, m_real b )
{
    return interval( a.m_start + b, a.m_end + b );
}

interval operator+( m_real b, interval const& a )
{
    return interval( a.m_start + b, a.m_end + b );
}

interval& operator-=( interval& a, m_real b )
{
    a.m_start -= b;
    a.m_end   -= b;

    return a;
}

interval operator-( interval const& a, m_real b )
{
    return interval( a.m_start - b, a.m_end - b );
}

interval& operator*=( interval& a, m_real b )
{
    a.m_start *= b;
    a.m_end   *= b;
    adjust_interval( a );

    return a;
}

interval operator*( interval const& a, m_real b )
{
    interval c( a.m_start * b, a.m_end * b );
    adjust_interval( c );

    return c;
}

interval operator*( m_real b, interval const& a )
{
    interval c( a.m_start * b, a.m_end * b );
    adjust_interval( c );

    return c;
}

interval& operator/=( interval& a, m_real b )
{
    a.m_start /= b;
    a.m_end   /= b;
    adjust_interval( a );

    return a;
}

interval operator/( interval const& a, m_real b )
{
    interval c( a.m_start / b, a.m_end / b );
    adjust_interval( c );

    return c;
}

interval& operator|=( interval& a, interval const& b )
{
    a.m_start = MIN( a.m_start, b.m_start );
    a.m_end   = MAX( a.m_end  , b.m_end   );

    return a;
}

interval operator|( interval const& a, interval const& b )
{
    return interval( MIN( a.m_start, b.m_start ), MAX( a.m_end, b.m_end ) );
}

int operator>>( interval const& a, m_real b )
{
    if ( a.m_start<b+EPS && b<a.m_end+EPS ) return TRUE;
                                   else return FALSE;
}

int operator<<( m_real b, interval const& a )
{
    if ( a.m_start<b+EPS && b<a.m_end+EPS ) return TRUE;
                                   else return FALSE;
}

int operator>>( interval const& a, interval const& b )
{
    if ( a.m_start<b.m_start+EPS && b.m_end<a.m_end+EPS ) return TRUE;
                                             else return FALSE;
}

int operator<<( interval const& b, interval const& a )
{
    if ( a.m_start<b.m_start+EPS && b.m_end<a.m_end+EPS ) return TRUE;
                                             else return FALSE;
}

interval& operator&=( interval& a, interval const& b )
{
    if ( a<<b );
    else if ( b<<a ) { a = b; }
    else if ( b.m_end<a.m_start ) { a *= 0.0; }
    else if ( a.m_end<b.m_start ) { a *= 0.0; }
    else if ( a.m_start<b.m_start ) { a.m_start = b.m_start; }
	else if ( b.m_end<a.m_end ) { a.m_end=b.m_end;}
    else { a.m_end = b.m_end; }
                               
    return a;
}
/*
interval& operator&=( interval& a, interval const& b )
{
    if ( a<<b );
    else if ( b<<a ) { a = b; }
    else if ( b.m_end<a.m_start ) { a *= 0.0; }
    else if ( a.m_end<b.m_start ) { a *= 0.0; }
    else if ( a.m_start<b.m_start ) { a.m_start = b.m_start; }
	else if ( b.m_end<a.m_end ) { a.m_end=b.m_end;}
    else { a.m_end = b.m_end; }
                               
    return a;
}
*/

interval operator&( interval const& a, interval const& b )
{
	interval c=a;
	c&=b;
	return c;

}

int operator&&( interval const& a, interval const& b )
{
    if ( a.m_start-EPS > b.m_end || a.m_end+EPS < b.m_start ) return FALSE;
                                                 else return TRUE;
}

interval operator^( interval const& a, m_real b )
{
    return interval( a.m_start - b, a.m_end + b );
}

void adjust_interval( interval& a )
{
    if ( a.m_start > a.m_end )
    {
        m_real temp = a.m_start;
        a.m_start = a.m_end;
        a.m_end = temp;
    }
}

m_real
interval::project( m_real d ) const
{
    if ( this->m_start > d ) return this->m_start;
    if ( this->m_end < d ) return this->m_end;
    return d;
}

m_real
interval::distance( m_real d ) const
{
    if ( this->m_start > d ) return this->m_start - d;
    if ( this->m_end < d ) return d - this->m_end;
    return 0;
}

m_real interval::uninterpolate( m_real input) const
{
	return (input-m_start)/(m_end-m_start);
}

void interval::calcRange(const vectorn& input)
{
	//!< find input range
	m_start=input.minimum();
	m_end=input.maximum();
}

void interval::calcQuartiles(const vectorn& input)	//!< find lower and upper quartile
{
	if(input.size()<4)
	{
		m_start=input.minimum();
		m_end=input.maximum();
		return;
	}

	vectorn sorted;
	intvectorn sortedIndex;
	sorted.sort(input, sortedIndex);
	
	m_start=sorted[sorted.size()/4];
	m_end=sorted[sorted.size()-1-sorted.size()/4];
}

void interval::scale(m_real s)	//!< scale the range from the mid point.
{
	m_real mid=(m_start+m_end)/2.0;

	m_start=mid+(m_start-mid)*s;
	m_end=mid+(m_end-mid)*s;
}

void interval::calcPercentages(const vectorn& input, m_real min, m_real max)
{
	if(input.size()<4)
	{
		m_start=input.minimum();
		m_end=input.maximum();
		return;
	}

	vectorn sorted;
	intvectorn sortedIndex;
	sorted.sort(input, sortedIndex);
	
	m_start=sorted[sorted.size()*min];
	m_end=sorted[sorted.size()-1-(sorted.size()*(1.0-max))];
}

void interval::calcOutlierFence(const vectorn& input, m_real safety)
{
	//!< find range excluding only extreme outliers
	//!< find range excluding mild outliers
	interval Q;
	Q.calcQuartiles(input);
	
	m_start=Q.start_pt()-safety*Q.len();
	m_end=Q.end_pt()+safety*Q.len();
}

m_real interval::rand_pt() const	// sampled from uniform distribution
{
	return ((m_real)rand()/(m_real)RAND_MAX)*(m_end-m_start)+m_start;
}
