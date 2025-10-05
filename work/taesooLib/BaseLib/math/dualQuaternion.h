#ifndef DUAL_QUTALSJASLJ_H_
#define DUAL_QUTALSJASLJ_H_
#pragma once

quater conjugate(quater const& q);

//---------------------------------------------------------------------------------------------------------------------
//	dualQuaternion
//---------------------------------------------------------------------------------------------------------------------

//
// A dual quaternion can do rigid transformations, i.e. rotation and translation.
// It does not support scaling. Based on quater.
//
// Unless noted otherwise, the <rotation,translation> pair is defined as
// first a rotation, followed by a translation.
//
// Notes:
// - Inspired by dual quaternion C++ code from Ladislav Kavan (kavanl1 AT fel.cvut.cz)
// - This is a reference implementation and not optimized for speed
//
class dualQuaternion
{
public:
						dualQuaternion();
						dualQuaternion(dualQuaternion const& inOther);
						dualQuaternion(quater const& inReal, quater const& inDual);
						dualQuaternion(double inRw, double inRx, double inRy, double inRz,
									    double inDw, double inDx, double inDy, double inDz);
	explicit			dualQuaternion(quater const& inRotation);
	explicit			dualQuaternion(vector3 const& inTranslation);
						dualQuaternion(vector3 const& inTranslation, quater const& inRotation);
	explicit			dualQuaternion(matrix4 const& inRigidTransform);
	dualQuaternion const&	operator=(dualQuaternion const& inRHS);
	dualQuaternion const&	operator=(matrix4 const& inRigidTransform);
	//dualQuaternion const&	operator=(rcMEulerRotation inRotationMatrix);

	// quater interface
	//bool				operator==(dualQuaternion const& inRHS) const;
	//bool				operator!=(dualQuaternion const& inRHS) const;
	dualQuaternion		operator+(dualQuaternion const& inRHS) const;
	dualQuaternion		operator-(dualQuaternion const& inRHS) const;
	dualQuaternion		operator-() const;
	friend dualQuaternion operator*(double inScale, dualQuaternion const& inRHS);
	dualQuaternion const&	negateIt();
	
	//bool	         	isEquivalent(dualQuaternion const& inOther, double inTolerance = 0.0001) const;
	dualQuaternion const&	scaleIt(double inScale);
	dualQuaternion 	normal() const;
	dualQuaternion const&	normalizeIt();
	dualQuaternion 	conjugate() const;
	dualQuaternion const&	conjugateIt();
	dualQuaternion 	inverse() const;
	dualQuaternion const&	invertIt();
	dualQuaternion		log() const;
	dualQuaternion		exp() const;
	matrix4				asMatrix() const;
	operator			matrix4() const;

	// Misc
	dualQuaternion const&	operator+=(dualQuaternion const& inRHS);
	dualQuaternion const&	operator*=(dualQuaternion const& inRHS);
	dualQuaternion const&	operator*=(double inScalar);
	dualQuaternion		operator*(dualQuaternion const& inRHS) const;
	double				dotReal(dualQuaternion const& inRHS) const;
	double				dotDual(dualQuaternion const& inRHS) const;
	double				dot(dualQuaternion const& inRHS) const;
	dualQuaternion		rotationNormalized() const;
	dualQuaternion		pluckerNormalized() const;
	quater			getRotation() const;
	vector3				getTranslation() const;
	vector3				getTranslationFromUnit() const;
	matrix4				asMatrixFromUnit() const;
	bool				checkPlucker() const;
	bool				isUnit() const;
	bool				hasRotation() const;
	bool				isPureTranslation() const;
	vector3				transform(vector3 const& inPoint) const;
	vector3				transformFromUnit(vector3 const& inPoint) const;
	dualQuaternion const&	setFromScrew(double inAngle, double inPitch, vector3 const& inDir, vector3 const& inMoment);
	void				toScrew(double& outAngle, double& outPitch, vector3& outDir, vector3& outMoment) const;

	// Do screw linear interpolation (the "slerp" for dual quaternions) for two unit dual quaternions
	static dualQuaternion		sScLERP(double inT, dualQuaternion const& inFrom, dualQuaternion const& inTo);

	// Do dual quaternion linear interpolation. Result is normalized afterwards.
	static dualQuaternion		sDLB(int inCount, const double inWeightList[], const dualQuaternion inDualQuatList[]);

	// Do dual quaternion iterative intrinsic interpolation up to given precision. Result is always normalized.
	static dualQuaternion		sDIB(int inCount, const double inWeightList[], const dualQuaternion inDualQuatList[], double inPrecision);

	// Static dualQuaternions
	static const dualQuaternion		zero;											///< R(0, 0, 0, 0), D(0,0,0,0)
	static const dualQuaternion		identity;										///< R(1, 0, 0, 0), D(0,0,0,0)

	quater			mReal;													///< Real quaternion part
	quater			mDual;													///< Dual quaternion part
};



//---------------------------------------------------------------------------------------------------------------------
//	dualQuaternion .inl
//---------------------------------------------------------------------------------------------------------------------

inline dualQuaternion::dualQuaternion()
{
}

inline dualQuaternion::dualQuaternion(dualQuaternion const& inOther) : 
	mReal(inOther.mReal),
	mDual(inOther.mDual)
{
}

inline dualQuaternion::dualQuaternion(quater const& inReal, quater const& inDual) : 
	mReal(inReal),
	mDual(inDual)
{
}

inline dualQuaternion::dualQuaternion(double inRw, double inRx, double inRy, double inRz, 
										double inDw, double inDx, double inDy, double inDz)
{
	mReal.x = inRx;	mReal.y = inRy;	mReal.z = inRz;	mReal.w = inRw;
	mDual.x = inDx;	mDual.y = inDy;	mDual.z = inDz;	mDual.w = inDw;
}

inline dualQuaternion::dualQuaternion(quater const& inRotation) :
	mReal(inRotation),
	mDual(0.0,0.0,0.0,0.0)
{
}

inline dualQuaternion::dualQuaternion(vector3 const& inTranslation) :
	mReal(quater(1,0,0,0)),
	mDual(0.0, 0.5*inTranslation.x, 0.5*inTranslation.y, 0.5*inTranslation.z)
{
}

inline dualQuaternion::dualQuaternion(vector3 const& inTranslation, quater const& inRotation) :
	mReal(inRotation),
	mDual(0.0, 0.5*inTranslation.x, 0.5*inTranslation.y, 0.5*inTranslation.z)
{
	mDual=mDual*mReal;
	//mDual .mult(mDual, mReal);
}

inline dualQuaternion::dualQuaternion(matrix4 const& inRigidTransform)
{
	*this = inRigidTransform;
}

inline dualQuaternion const& dualQuaternion::operator=(dualQuaternion const& inRHS)
{
	mReal = inRHS.mReal;
	mDual = inRHS.mDual;
	return *this;
}

/*
inline dualQuaternion const& dualQuaternion::operator=(rcMEulerRotation inRotationMatrix)
{
	mReal = inRotationMatrix;
	mDual = quater(0.0,0.0,0.0,0.0);
}*/

/*
inline bool dualQuaternion::operator==(dualQuaternion const& inRHS) const
{
	return mReal==inRHS.mReal && mDual==inRHS.mDual;
}

inline bool dualQuaternion::operator!=(dualQuaternion const& inRHS) const
{
	return !((*this)==inRHS);
}*/

inline double dualQuaternion::dotReal(dualQuaternion const& inRHS) const
{
	return mReal%inRHS.mReal;
}

inline double dualQuaternion::dotDual(dualQuaternion const& inRHS) const
{
	return mDual%inRHS.mDual;
}

inline double dualQuaternion::dot(dualQuaternion const& inRHS) const
{
	return dotReal(inRHS) + dotDual(inRHS);
}

inline dualQuaternion dualQuaternion::operator-() const
{
	return dualQuaternion(-mReal, -mDual);
}

inline dualQuaternion const& dualQuaternion::negateIt()
{
	mReal*=-1;
	mDual*=-1;
	return *this;
}

inline dualQuaternion dualQuaternion::operator+(dualQuaternion const& inRHS) const
{
	return dualQuaternion(mReal+inRHS.mReal, mDual+inRHS.mDual);
}

inline dualQuaternion dualQuaternion::operator-(dualQuaternion const& inRHS) const
{
	return dualQuaternion(mReal-inRHS.mReal, mDual-inRHS.mDual);
}

/*
inline bool dualQuaternion::isEquivalent(dualQuaternion const& inOther, double inTolerance) const
{
	return mReal.isEquivalent(inOther.mReal, inTolerance) && mDual.isEquivalent(inOther.mDual, inTolerance);
}*/

inline dualQuaternion const& dualQuaternion::scaleIt(double inScale)
{
	mReal*=inScale;
	mDual*=inScale;
	return *this;
}

inline dualQuaternion dualQuaternion::rotationNormalized() const
{
	double oo_magn = 1.0/sqrt(mReal% mReal);
	return dualQuaternion(oo_magn*mReal, oo_magn*mDual);
}

inline dualQuaternion dualQuaternion::pluckerNormalized() const
{
	double oo_magn_sqr = 1.0/(mReal% mReal);
	return dualQuaternion(mReal, mDual - (mReal% mDual*oo_magn_sqr)*mReal);
}

inline bool dualQuaternion::checkPlucker() const
{
	// Test for Pl?ker condition. Dot between real and dual part must be 0
	return fabs(mReal%mDual)<1e-5;
}

inline bool dualQuaternion::isUnit() const
{
	// Real must be unit and plucker condition must hold
	return (fabs(mReal%mReal-1.0)<1e-5) && checkPlucker();
}

inline bool dualQuaternion::hasRotation() const
{
	assert(isUnit());
	return fabs(mReal.w)<0.999999f;
}

inline bool dualQuaternion::isPureTranslation() const
{
	return !hasRotation();
}

inline quater dualQuaternion::getRotation() const
{
	assert(isUnit());
	return mReal;
}

inline vector3 dualQuaternion::getTranslationFromUnit() const
{
	assert(isUnit());
	quater qeq0 ;
	qeq0.mult(mDual, ::conjugate(mReal));
	return vector3(2.0*qeq0.x, 2.0*qeq0.y, 2.0*qeq0.z);
}

inline vector3 dualQuaternion::getTranslation() const
{	
	if (isUnit())
	{
		return getTranslationFromUnit();
	}
	else
	{
		double nq		= mReal% mReal;
		double scale	= -2.0/nq;

		double rw		= mReal.w; double rx = mReal.x; double ry = mReal.y; double rz = mReal.z;
		double dw		= mDual.w; double dx = mDual.x; double dy = mDual.y; double dz = mDual.z;

		double tx		= (dw*rx - rw*dx + dy*rz - ry*dz)*scale;
		double ty		= (dw*ry - dx*rz + rx*dz - rw*dy)*scale;
		double tz		= (dw*rz - rx*dy - rw*dz + dx*ry)*scale;

		// This can also be written as:
		//	quater qeq0= mulQuat(mDual, mReal.conjugate()) - mulQuat(mReal, mDual.conjugate());
		//	double tx		= qeq0.x/nq;
		//	double ty		= qeq0.y/nq;
		//	double tz		= qeq0.z/nq;

		return vector3(tx,ty,tz);
	}
}

inline dualQuaternion dualQuaternion::normal() const
{
	dualQuaternion dq(*this);
	return dq.normalizeIt();
}

inline dualQuaternion const& dualQuaternion::normalizeIt()
{
	// NB: the order of normalizations does not matter
	*this = rotationNormalized().pluckerNormalized();
	return *this;
}

inline dualQuaternion dualQuaternion::conjugate() const
{
	dualQuaternion dq(*this);
	return dq.conjugateIt();
}

inline dualQuaternion const& dualQuaternion::conjugateIt()
{
	mReal=::conjugate(mReal);
	mDual=::conjugate(mDual);
	return *this;
}

inline dualQuaternion dualQuaternion::inverse() const
{
	dualQuaternion dq(*this);
	return dq.invertIt();
}

inline dualQuaternion operator*(double inScale, dualQuaternion const& inRHS)
{
	return dualQuaternion(inScale*inRHS.mReal, inScale*inRHS.mDual);
}

inline dualQuaternion const& dualQuaternion::operator+=(dualQuaternion const& inRHS)
{
	mReal = mReal + inRHS.mReal;
	mDual = mDual + inRHS.mDual;
	return *this;
}

inline dualQuaternion const& dualQuaternion::operator*=(double inScalar)
{
	return scaleIt(inScalar);
}

inline dualQuaternion const& dualQuaternion::operator*=(dualQuaternion const& inRHS)
{
	*this = (*this)* inRHS ;
	return *this;
}

inline dualQuaternion dualQuaternion::operator*(dualQuaternion const& inRHS) const
{
	return dualQuaternion(mReal*inRHS.mReal, mReal*inRHS.mDual + mDual*inRHS.mReal);
}

inline vector3 dualQuaternion::transform(vector3 const& inPoint) const
{
	matrix4 mat = asMatrix();
	return mat*inPoint ;
}

inline dualQuaternion::operator matrix4() const
{
	return asMatrix();
}

#endif
