#ifndef BASELIB_INTERVAL_H_
#define BASELIB_INTERVAL_H_
#pragma once
class interval
{
  private:
    m_real m_start, m_end;
    
    // negation
    friend interval operator-( interval const& );

    // addtion
    friend interval& operator+=( interval&, m_real );
    friend interval  operator+( m_real, interval const& );
    friend interval  operator+( interval const&, m_real );

    // subtraction
    friend interval& operator-=( interval&, m_real );
    friend interval  operator-( interval const&, m_real );

    // multiplication by scalar
    friend interval& operator*=( interval&, m_real );
    friend interval  operator*( interval const&, m_real );
    friend interval  operator*( m_real, interval const& );

    // division by scalar
    friend interval& operator/=( interval&, m_real );
    friend interval  operator/( interval const&, m_real );

    // inclusion operation
    friend int   operator>>( interval const&, m_real );
    friend int   operator<<( m_real, interval const& );

    // inclusion operation
    friend int   operator>>( interval const&, interval const& );
    friend int   operator<<( interval const&, interval const& );

    // or operation
    friend interval& operator|=( interval&, interval const& );
    friend interval  operator|( interval const&, interval const& );

    // and operation
    friend interval& operator&=( interval&, interval const& );
    friend interval  operator&( interval const&, interval const& );

    // logical-and operation
    friend int       operator&&( interval const&, interval const& );

    // expansion
    friend interval  operator^( interval const&, m_real );

    // axiliary functions
    friend void adjust_interval( interval& );

    // stream
    friend std::ostream& operator<<( std::ostream&, interval const& );
    friend std::istream& operator>>( std::istream&, interval& );

  public:
    // constructors
    interval( ) { m_start = m_end = 0.0; }
    interval( m_real a ) { m_start = m_end = a; }
    interval( m_real a, m_real b ) { m_start = MIN(a,b);
                                     m_end   = MAX(a,b); }

	void setValue(m_real a, m_real b) 	{ m_start = MIN(a,b); m_end   = MAX(a,b); }
    // inquiry functions
    m_real start_pt() const { return m_start ; }
    m_real end_pt() const { return m_end; }
	m_real& start()		{ return this->m_start ;}
	m_real& end()		{ return this->m_end;}
	m_real& left()		{ return m_start ;}
	m_real& right()		{ return m_end;}

	bool isInside(m_real value) const;
    m_real mid_pt() const { return (m_start +m_end)/2.0f; }
	m_real rand_pt() const;	// sampled from uniform distribution

    m_real interpolate( m_real t ) const { return (1-t)*m_start  + t*m_end; }
	m_real uninterpolate( m_real value) const;	//!< find t such that interpolate(t)==value
	
	m_real len( ) const { return m_end - m_start ; }

	m_real distance( m_real ) const;
    m_real project( m_real ) const;

	void scale(m_real s);	//!< scale the range from the mid point.
	void enlarge(m_real point);	//!< enlarge interval so that it include the point
	void expand(m_real epsilon);	//!< min=min-epsilon, max=max+epsilon
	
	void calcRange(const vectorn& input);
	void calcQuartiles(const vectorn& input);	//!< find lower and upper quartile
	void calcPercentages(const vectorn& input, m_real min=0.05, m_real max=0.95);
	/**
	 * find range of input excluding outliers
	 * \param input 
	 * \param safety safety=1.5 for excluding all mild outliers
	 *               safety=3 for excluding only extreme outlier
	 */
	void calcOutlierFence(const vectorn& input, m_real safety=3);	
};
#endif
