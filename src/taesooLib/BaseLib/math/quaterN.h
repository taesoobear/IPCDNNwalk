#pragma once

#include "quater.h"
#include "template_math.h"
class quaterNView;
class quaterN : public _tvectorn<quater, m_real  >
{	
protected:
	quaterN(m_real* ptrr, int size, int stride):_tvectorn<quater, m_real  >(ptrr,size,stride){}
public:
	quaterN();
	quaterN(int n);
	
	// 값을 카피해서 받아온다.	

	template <class VecType>
	quaterN(const VecType& other)	{ assign(other);}
	quaterN(const quaterN& other)	{ assign(other);}

	~quaterN();


	// L-value로 사용될수 있는, reference array를 만들어 return 한다. 
	// ex) v.range(0,2).derivative(..)
	quaterNView	range(int start, int end, int step=1);

	//void setSize(int n);
	int rows() const					{ return size();}
	quater& row(int i) const		{ return value(i);}
	//quater& operator[](int i) const	{ return row(i);}

	vectornView		w() const;
	vectornView		x() const;
	vectornView		y() const;
	vectornView		z() const;
	vectornView		column(int i) const;

	void assign(const quaterN& other);

	// void derivative(const quaterN& other);

	/**
	 * Online 으로 stitch된 경우의 displacementMap을 계산한다.
	 * 
	 * ____ src motion
	 *     ++++ add motion
	 *     **** make displacementmap
	 *
	 * \param sq1 srcmotion[numFrame-2]
	 * \param sq2 srcmotion[numFrame-1]
	 * \param aq1 addMotion[0]
	 * \param aq2 addMotion[1]
	 * \param duration length of displacementmap
	 */
	void displacementOnline(const quater& sq1, const quater& sq2, const quater& aq1, const quater& aq2, int duration);
	void displacement(const quater& sp1, const quater& sp2, const quater& ap2, const quater& ap3, int start, int end);
	/**
	 * ab와 cd를 연결하는 hermite curve를 만든다. 중간은 duration으로 주어진다.
	 *   ab
	 * ----
	 *    -- duration -- 
	 *                 +++++++
	 *                 cd
	 *    ************** hermite range (size=duration)
	 */
	void hermite(const quater& a, const quater& b, int duration, const quater& c, const quater& d, float interval=1.f);
	/// prefer identity. see source code for more detail.
	void hermite0(const quater& a, const quater& b, int duration, const quater& c, const quater& d, float interval=1.f);
	void hermite_mid(const quater& a, const quater& b, int duration, const quater& c, const quater& d, const quater& qi, float interval=1.f);
	/// size=duration a-duration-b
	void transition(const quater& a, const quater& b, int duration);
	/// prefer identity. see source code for more detail.
	void transition0(const quater& a, const quater& b, int duration);

	void c0stitch(int discontinuity);
	void c1stitch(int discontinuity);	// use hermite curve();
	void hermite(int discontinuity);
	void decomposeStitch(int discontinuity);
	void linstitch(int discontinuity);

	// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
	void linstitch(quaterN const& a, quaterN const& b);
	void c0stitch(quaterN const& a, quaterN const& b);
	void c0stitchOnline(quaterN const& a, quaterN const& b);
	void c0stitchForward(quaterN const& a, quaterN const& b);
	void c1stitch(quaterN const& a, quaterN const& b);	// use hermite curve();

	void decompose(quaterN& rotY, quaterN& offset) const;
	void combine(const quaterN& rotY, const quaterN& offset);
	
	void align();

	template <class VecType>
	void operator=(const VecType& other)	{ _tvectorn<quater, m_real>::assign(other);}
	void operator=(const quaterN& other)	{ _tvectorn<quater, m_real>::assign(other);}

private:
//	void release();
//	quater* m_aQuater;
//	int m_nSize;
//	int m_nCapacity;
};

class quaterNView :public quaterN
{
public:
	// L-value로 사용될수 있는, reference array로 만든다. 
	quaterNView (m_real* ptrr, int size, int stride);	

	quaterNView ():quaterN()						{Msg::error("do not call this");}
	// 값을 reference로 받아온다.
	template <class VecType>
	quaterNView(const VecType& other)	{ assignRef(other);}
	// The following function is necessary! The above function covers all but quaterNView.
	quaterNView(const quaterNView& other)	{ assignRef(other);}

	~quaterNView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	void operator=(const VecType& other)	{ _tvectorn<quater, m_real>::assign(other);}
	void operator=(const quaterNView& other){ _tvectorn<quater, m_real>::assign(other);}

	/*
    // enable const to non-const casting (so that mat.range(...) can be directly used as a function argument.)
    inline operator matrixn& () const { return (matrixn&)(*this);}
	// doesn't work in some gnu compilers
	*/
	quaterN& lval() const { return (quaterN&)(*this);}
};
