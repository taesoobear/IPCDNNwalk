#pragma once

#include "vector3.h"
#include "vector3N.h"

/// Uniform Cubic Spline. Implementing http://mathworld.wolfram.com/CubicSpline.html by taesoo
class UniformSpline
{
public:

	UniformSpline(const matrixn& points, int start=0, int end=INT_MAX);
	~UniformSpline(){};

	matrixn DT;	//!< D transpose
	matrixn y;
	vectorn tridM[3];	//!< tridiagonal matrix.
	int dim;	// dimension of input vectors
	int n;	// number of control points

	void getCurve(matrixn& points, int ndiv);
	void getSecondDeriv(matrixn& points);
};

/// Nonuniform Cubic Interpolating Spline. Implemented by taesoo
class NonuniformSpline
{
	
public:
	struct boundaryCondition
	{
		enum bcT { ZERO_VEL, ZERO_ACC , VEL} ;
		boundaryCondition(bcT type):mType(type){}
		boundaryCondition(bcT type, vectorn const& con):mType(type), mConstraint(con){}
		~boundaryCondition(){}
		bcT mType;
		vectorn mConstraint;
	};

	struct zeroVel:public boundaryCondition
	{
		zeroVel():boundaryCondition(boundaryCondition::ZERO_VEL){}
	};
	struct zeroAcc:public boundaryCondition
	{
		zeroAcc():boundaryCondition(boundaryCondition::ZERO_ACC){}
	};
	struct velocity:public boundaryCondition
	{
		velocity(const vectorn& vel):boundaryCondition(boundaryCondition::VEL,vel){}
	};

	// keytime must be sorted array
	NonuniformSpline(vectorn const& keytime, const matrixn& controlpoints, const boundaryCondition &bc0, const boundaryCondition &bcn);
	NonuniformSpline(vectorn const& keytime, const matrixn& controlpoints);	// assumes zero_acc
	NonuniformSpline(vectorn const& keytime, const matrixn& controlpoints, boundaryCondition::bcT bc);
	~NonuniformSpline(){};

	matrixn DT;	//!< D transpose
	matrixn y;
	vectorn tridM[3];	//!< tridiagonal matrix.
	int dim;	// dimension of input vectors
	int n;	// number of control points

	vectorn keytime;
	vectorn len;

	// time은 keytime[0]보다 크거나 같고, keytime[keytime.size()-1]보다 작거나 같은 범위에서만 동작한다.
	void getCurve(vectorn const& time, matrixn& points);
	void getFirstDeriv(vectorn const& time, matrixn& points);
	void getSecondDeriv(vectorn const& time, matrixn& points);
private:
	void __init(vectorn const& keytime, const matrixn& controlpoints, const boundaryCondition &bc0, const boundaryCondition &bcn);
};

/// Cubic spline with variable knot. downloaded from internet. Taesoo modified...
class Spline
{
public:
	matrixn P;
	matrixn A;
	matrixn B;
	matrixn C;
	vectorn k;
	vectorn Mat[3];	// tridiagonal matrix

	int NP;

	enum { UNIFORM_KNOT, CHORD_LENGTH };
	Spline(const matrixn& points, int start=0, int end=INT_MAX, int knotSpacingMethod=UNIFORM_KNOT);
	~Spline() {};

	void getCurve(matrixn& points, int ndiv=0);	// ndiv: the number of divisions in a chord. if ndiv==0 it automatically determines ndiv

private:
	struct Curve
	{
		vectorn A;
		vectorn B;
		vectorn C;
		int    Ndiv;

		void putCurve(const vectorn& a, const vectorn& b, const vectorn& c, int ndiv=0);
		void getCurve(const vectorn& p, matrixn& points, int& PointCount);
		int getCount();
	};

	void generate();	// calc control points
	static void MatrixSolve(vectorn& B, vectorn Mat[3], int NP);
};

/* - buggy
// Least Square Fitting 
class UniformBSpline
{
public:
	/// knot spacing should be uniform. not duplicate knot. (I will do that automatically.) domain: [0~end-start]
	UniformBSpline(const vectorn & knot, const matrixn& points, int start=0, int end=INT_MAX);
	~UniformBSpline(){};

	matrixn m_vSamples;
	matrixn m_vCP;	// control points
	vectorn m_vKnot;	

	m_real basis(m_real d);	// You can use any kind of basis. Currently cubic bspline bases

};*/


class BSpline
{
public:
	// Magic Software, Inc.
// http://www.magic-software.com
// http://www.wild-magic.com
// Copyright (c) 2004.  All Rights Reserved
//
// The Wild Magic Library (WML) source code is supplied under the terms of
// the license agreement http://www.magic-software.com/License/WildMagic.pdf
// and may not be copied or disclosed except in accordance with the terms of
// that agreement.

	class Basis
	{
	public:
		Basis ();

		// Open uniform or periodic uniform.  The knot array is internally
		// generated with equally spaced elements.
		Basis (int iNumCtrlPoints, int iDegree, bool bOpen);
		void Create (int iNumCtrlPoints, int iDegree, bool bOpen);

		// Open nonuniform.  The knot array must have n-d elements.  The elements
		// must be nondecreasing.  Each element must be in [0,1].  The caller is
		// responsible for deleting afKnot.  An internal copy is made, so to
		// dynamically change knots you must use the 'm_real& Knot (int i)'
		// function.
		Basis (const vectorn& afKnot, int iDegree);
		void Create (const vectorn& afKnot, int iDegree);

		virtual ~Basis ();

		int GetNumCtrlPoints () const		{	return m_iNumCtrlPoints;}
		int GetDegree () const				{   return m_iDegree;}
		bool IsOpen () const				{   return m_bOpen;}
		bool IsUniform () const				{   return m_bUniform;}


		// The knot values can be changed only if the basis function is nonuniform
		// and the input index is valid (0 <= i <= n-d-1).  If these conditions
		// are not satisfied, the return value is undefined.
		m_real& Knot (int i);

		// access basis functions and their derivatives
		m_real GetD0 (int i) const			{ return m_aafBD0[m_iDegree][i];}
		m_real GetD1 (int i) const			{ return m_aafBD1[m_iDegree][i];}
		m_real GetD2 (int i) const			{ return m_aafBD2[m_iDegree][i];}
		m_real GetD3 (int i) const			{ return m_aafBD3[m_iDegree][i];}

		// evaluate basis functions and their derivatives
		void Compute (m_real fTime, unsigned int uiOrder, int& riMinIndex,
			int& riMaxIndex) const;

	protected:
		int Initialize (int iNumCtrlPoints, int iDegree, bool bOpen);
		m_real** Allocate () const;
		void Deallocate (m_real** aafArray);

		// Determine knot index i for which knot[i] <= rfTime < knot[i+1].
		int GetKey (m_real& rfTime) const;

		int m_iNumCtrlPoints;    // n+1
		int m_iDegree;           // d
		vectorn m_afKnot;          // knot[n+d+2]
		bool m_bOpen, m_bUniform;

		// Storage for the basis functions and their derivatives first three
		// derivatives.  The basis array is always allocated by the constructor
		// calls.  A derivative basis array is allocated on the first call to a
		// derivative member function.
		m_real** m_aafBD0;             // bd0[d+1][n+d+1]
		mutable m_real** m_aafBD1;     // bd1[d+1][n+d+1]
		mutable m_real** m_aafBD2;     // bd2[d+1][n+d+1]
		mutable m_real** m_aafBD3;     // bd3[d+1][n+d+1]

	};



    // Construction and destruction.  The caller is responsible for deleting
    // the input arrays if they were dynamically allocated.  Internal copies
    // of the arrays are made, so to dynamically change control points or
    // knots you must use the 'SetControlPoint', 'GetControlPoint', and
    // 'Knot' member functions.

    // Uniform spline.  The number of control points is n+1 >= 2.  The degree
    // of the B-spline is d and must satisfy 1 <= d <= n.  The knots are
    // implicitly calculated in [0,1].  If bOpen is 'true', the spline is
    // open and the knots are
    //   t[i] = 0,               0 <= i <= d
    //          (i-d)/(n+1-d),   d+1 <= i <= n
    //          1,               n+1 <= i <= n+d+1
    // If bOpen is 'false', the spline is periodic and the knots are
    //   t[i] = (i-d)/(n+1-d),   0 <= i <= n+d+1
    // If bLoop is 'true', extra control points are added to generate a closed
    // curve.  For an open spline, the control point array is reallocated and
    // one extra control point is added, set to the first control point
    // C[n+1] = C[0].  For a periodic spline, the control point array is
    // reallocated and the first d points are replicated.  In either case the
    // knot array is calculated accordingly.

    BSpline (const matrixn& aCtrlPoints, int iDegree, bool bLoop=false, bool bOpen=true);

    // Open, nonuniform spline.  The knot array must have n-d elements.  The
    // elements must be nondecreasing.  Each element must be in [0,1].
    BSpline (const matrixn& aCtrlPoints, int iDegree, bool bLoop, const vectorn& afKnot);

    virtual ~BSpline ();

	int GetNumCtrlPoints () const	{		return m_iNumCtrlPoints;	}
	int GetDegree () const			{		return m_kBasis.GetDegree();	}
	bool IsOpen () const			{		return m_kBasis.IsOpen();	}
	bool IsUniform () const			{		return m_kBasis.IsUniform();	}
	bool IsLoop () const			{		return m_bLoop;	}

    // Control points may be changed at any time.  The input index should be
    // valid (0 <= i <= n).  If it is invalid, the return value is undefined.
    void SetControlPoint (int i, const vectorn& rkCtrl);
    //const vectorn& GetControlPoint (int i) const;
	inline vectornView GetControlPoint (int i) const	{return m_akCtrlPoint.row(i);}

    // The knot values can be changed only if the basis function is nonuniform
    // and the input index is valid (0 <= i <= n-d-1).  If these conditions
    // are not satisfied, the return value is undefined.
    m_real& Knot (int i);

    // The spline is defined for 0 <= t <= 1.  If a t-value is outside [0,1],
    // an open spline clamps t to [0,1].  That is, if t > 1, t is set to 1;
    // if t < 0, t is set to 0.  A periodic spline wraps to to [0,1].  That
    // is, if t is outside [0,1], then t is set to t-floor(t).
	void GetPosition (m_real fTime, vectorn& kPos) const				{Get(fTime,&kPos,NULL,NULL,NULL);}
	void GetFirstDerivative (m_real fTime, vectorn& kDer1) const		{Get(fTime,NULL,&kDer1,NULL,NULL);}
	void GetSecondDerivative (m_real fTime, vectorn& kDer2) const	{Get(fTime,NULL,NULL,&kDer2,NULL);}
	void GetThirdDerivative (m_real fTime, vectorn& kDer3) const		{Get(fTime,NULL,NULL,NULL,&kDer3);}

    // If you need position and derivatives at the same time, it is more
    // efficient to call these functions.  Pass the addresses of those
    // quantities whose values you want.  You may pass NULL in any argument
    // whose value you do not want.
    void Get (m_real fTime, vectorn* pkPos, vectorn* pkDer1,
        vectorn* pkDer2, vectorn* pkDer3) const;

    // Access the basis function to compute it without control points.  This
    // is useful for least squares fitting of curves.
    Basis& GetBasis ();

protected:
    // Replicate the necessary number of control points when the Create
    // function has bLoop equal to true, in which case the spline curve must
    // be a closed curve.
	void CreateControl (const matrixn& akCtrlPoint);

    int m_iNumCtrlPoints;
    matrixn m_akCtrlPoint;  // ctrl[n+1]
    bool m_bLoop;
    Basis m_kBasis;
    int m_iReplicate;  // the number of replicated control points
};
