#ifndef _VECTOR3N_H_
#define _VECTOR3N_H_

#if _MSC_VER > 1000
#pragma once
//#pragma message("Compiling vector3N.h - this should happen just once per project.\n")
#endif

#include "vector3.h"
class matrixn;
class vector3NView;


class vector3N : public _tvectorn<vector3, m_real>
{
protected:
	vector3N(m_real * ptrr, int size, int stride):_tvectorn<vector3, m_real>(ptrr,size,stride){}
public:
	vector3N();
	vector3N(int n);

	vector3N(const vector3NView& other);
	vector3N(const vector3N& other)	{ assign(other);}
	vector3N(const matrixnView& other)	{ assign(other);}
	vector3N(const matrixn& other)	{ assign(other);}

	~vector3N();

	vector3 sampleRow(m_real criticalTime) const;
	int rows() const						{ return size();}
	vector3& row(int i) const			{ return value(i);}
	vector3& operator[](int i) const	{ return row(i);}
	void assign(const vector3N& other);
	void assign(const matrixn& other);

	// L-value로 사용될수 있는, reference array를 만들어 return 한다.
	// ex) v.range(0,2).derivative(..)
	vector3NView	range(int start, int end, int step=1);

	// L-value로 사용될수 있는 reference vector 를 return한다.
	vectornView		x() const;
	vectornView		y() const;
	vectornView		z() const;
	vectornView		column(int i) const;

	void translate(const vector3& trans);
	void rotate(const quater& q);		 //!< Data 전체를 Rotate한다. root orientation quat앞에 곱해져서 제일 나중에

	void rotate(const vector3& center, const quater&q)
	{
		translate(-center);
		rotate(q);
		translate(center);
	}

	vector3N operator*(m_real) const;

	// void derivative(const vector3N& other);

	/**
	 * Online 으로 stitch된 경우의 displacementMap을 계산한다.
	 *
	 * ____ src motion
	 *     ++++ add motion
	 *     **** make displacementmap
	 *
	 * \param sp1 srcmotion[numFrame-2]
	 * \param sp2 srcmotion[numFrame-1]
	 * \param ap1 addMotion[0]
	 * \param ap2 addMotion[1]
	 * \param duration length of displacementmap
	 */
	void displacementOnline(const vector3& sp1, const vector3& sp2, const vector3& ap1, const vector3& ap2, int duration);

	/**
	 * Offline 으로 stitch된 경우의 displacementMap을 계산한다.
	 *
	 * ____ src motion
	 *     ++++ add motion
	 *   **** make displacementmap
	 *
	 * \param sp1 srcmotion[numFrame-2]
	 * \param sp2 srcmotion[numFrame-1]
	 * \param ap1 addMotion[0]
	 * \param ap2 addMotion[1]
	 * \param start: For example, in the above picture, start = -2
	 * \param end : ... end=2
	 */
	void displacement(const vector3& sp1, const vector3& sp2, const vector3& ap1, const vector3& ap2, int start, int end);

	/**
	 * Offline 으로 stitch된 경우의 displacementMap을 계산한다. Better stitching.
	 *
	 * _____ src motion
	 *      +++++ add motion
	 *  ******** make displacementmap
	 * (sps 랑 spe는 중간의 speed를 보다 잘 estimate하기 위해 쓰임)
	 * \param sps srcmotion[numFrame+start]
	 * \param sp1 srcmotion[numFrame-2]
	 * \param sp2 srcmotion[numFrame-1]
	 * \param ap1 addMotion[0]
	 * \param ap2 addMotion[1]
	 * \param ape addMotion[end-1]
	 * \param start: For example, in the above picture, start = -4
	 * \param end : ... end=4
	 */
	void displacement(const vector3& sps, const vector3& sp1, const vector3& sp2,
						const vector3& ap1, const vector3& ap2, const vector3& spe, int start, int end);

	/**
	 * ab와 cd를 연결하는 hermite curve를 만든다. 중간은 duration으로 주어진다.
	 *   ab
	 * ----
	 *    -- duration --
	 *                 +++++++
	 *                 cd
	 *    ************** hermite range (size=duration)
	 */
	void hermite(const vector3& a, const vector3& b, int duration, const vector3& c, const vector3& d);
	void transition(const vector3& a, const vector3& b, int duration);


	//////////////////////////////////////////////////////////////////////////
	// stitch 계열 함수들. void xxx(int discontinuity)
	// this 가 discontinuity있는 array를 가정하고, 연결.
	//////////////////////////////////////////////////////////////////////////

	static void stitchTest(void (vector3N::*func)(int discontinuity), const char* filename);
	// stitch this at center. (even sized array has discontinuity at the center of this.)
	void c0stitch(int discontinuity);
	void c1stitch(int discontinuity);	// use hermite curve();
	// linear system.
	void linstitch(int discontinuity);

	// 시작 두개점과 끝 두개점만 사용.
	void hermite(int discontinuity);

	// a의 마지막 프레임이 b의 첫프레임과 동일해지도록 stitch. 최종 길이는 a.size()+b.size()-1이 된다.
	void linstitch(vector3N const& a, vector3N const& b);
	void c0stitch(vector3N const& a, vector3N const& b);
	operator vector3*() const	{ return dataPtr();}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	void operator=(const VecType& other)	{ _tvectorn<vector3, m_real>::assign(other);}
	void operator=(const vector3N& other)	{ _tvectorn<vector3, m_real>::assign(other);}
};


class vector3NView :public vector3N
{
public:
	// L-value로 사용될수 있는, reference array로 만든다.
	vector3NView (m_real * ptrr, int size, int stride);

	vector3NView ():vector3N()							{Msg::error("do not call this");}

	// 값을 reference로 받아온다.
	template <class VecType>
	vector3NView(const VecType& other)		{ assignRef(other);}
	vector3NView(const vector3NView& other)	{ assignRef(other);}
	~vector3NView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	void operator=(const VecType& other)		{ _tvectorn<vector3, m_real>::assign(other);}
	void operator=(const vector3NView& other)	{ _tvectorn<vector3, m_real>::assign(other);}
};
#endif
