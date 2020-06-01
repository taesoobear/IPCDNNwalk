#pragma once
#include "vectorn.h"
class intervalN
{
  private:
    vectorn m_vStart;
	vectorn m_vEnd;
    
    // negation
    friend intervalN operator-( intervalN const& );

    // addtion
    friend intervalN& operator+=( intervalN&, vectorn );
    friend intervalN  operator+( vectorn, intervalN const& );
    friend intervalN  operator+( intervalN const&, vectorn );

    // subtraction
    friend intervalN& operator-=( intervalN&, vectorn );
    friend intervalN  operator-( intervalN const&, vectorn );

    // multiplication by scalar
    friend intervalN& operator*=( intervalN&, m_real );
    friend intervalN  operator*( intervalN const&, m_real );
    friend intervalN  operator*( m_real, intervalN const& );

    // division by scalar
    friend intervalN& operator/=( intervalN&, m_real );
    friend intervalN  operator/( intervalN const&, m_real );

    // inclusion operation
    friend int   operator>>( intervalN const&, vectorn const&  );
    friend int   operator<<( vectorn const& , intervalN const& );

    // inclusion operation
    friend int   operator>>( intervalN const&, intervalN const& );
    friend int   operator<<( intervalN const&, intervalN const& );

    // or operation
    friend intervalN& operator|=( intervalN&, intervalN const& );
    friend intervalN  operator|( intervalN const&, intervalN const& );

    // and operation
    friend intervalN& operator&=( intervalN&, intervalN const& );
    friend intervalN  operator&( intervalN const&, intervalN const& );

    // logical-and operation
    friend int       operator&&( intervalN const&, intervalN const& );

    // expansion
    friend intervalN  operator^( intervalN const&, vectorn );

    // axiliary functions
    friend void adjust_intervalN( intervalN& );

    // stream
//    friend ostream& operator<<( ostream&, intervalN const& );
  //  friend istream& operator>>( istream&, intervalN& );

  public:
    // constructors
	 intervalN( ) { }
    explicit intervalN( vectorn a ) { m_vStart  = m_vEnd = a; };
    intervalN( vectorn const &a, vectorn const &b );// { m_vStart.min(a,b);   m_vEnd.max(a,b); };

	void setSize(int n)			{ m_vStart.setSize(n); m_vEnd.setSize(n);}
	int size() const			{ ASSERT(m_vStart.size()==m_vEnd.size()); return m_vStart.size();}
	void initEmpty();
	void enlarge(const vectorn& point);	//!< enlarge interval so that it include the point
	void expand(const vectorn& epsilon);	//!< min=min-epsilon, max=max+epsilon
    // inquiry functions
	
	m_real start(int index) const {return m_vStart[index];}
	m_real end(int index) const	{return m_vEnd[index];}
	const vectorn& start() const { return m_vStart ; }
	const vectorn& end() const { return m_vEnd; }
	void assign(const vectorn& a, const vectorn& b);

	vectorn& start_ref() { return m_vStart ; }
	vectorn& end_ref() { return m_vEnd; }
    
    void mid_pt(vectorn& midpoint);
	void rand_pt(vectorn& randpoint);	// sampling using uniform distribution.

	
	void interpolate( m_real t,vectorn& result ) const;
	void interpolate( const vectorn& t,vectorn& result ) const;
	void uninterpolate( const vectorn& result, vectorn& t) const;

	/// nick name of interpolate function
	void normalize(const vectorn& original, vectorn& normalized) const		{ uninterpolate(original, normalized); }
	void unnormalize(const vectorn& normalized, vectorn& original) const	{ interpolate(normalized, original); }

    void getRange(vectorn & range ) const;
	void calcRange(const matrixn& input);	//!< find input range
	void normalizeRange(const vectorn& aspectRatio);
	/**
	 * find range of input excluding outliers
	 * \param input 
	 * \param safety safety=1.5 for excluding all mild outliers
	 *               safety=3 for excluding only extreme outlier
	 */
	void calcOutlierFence(const matrixn& input, m_real safety=1.5);	

	m_real distance( vectorn const& d ) const;
	
	void project( vectorn const& d, vectorn& projected, vectorn & ProjectAxis ) const;
    void project( vectorn const& d, vectorn& projected) const { vectorn ProjectAxis; project(d, projected, ProjectAxis);}
	//vectorn project( vectorn const& d, vectorn & ProjectAxis ) const;

	interval operator[](int index) const	{ interval i(start(index), end(index)); return i;}
	interval getInterval(int index)	const 	{ return (*this)[index];}
};
