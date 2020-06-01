#include "stdafx.h"
#include "../../BaseLib/math/mathclass.h"
#include "../../BaseLib/math/quater.h"
#include "../../BaseLib/math/vector3.h"
#include "dualQuaternion.h"
// dualQuaternion.cpp - Copyright (C) Guido de Haan 2006-2007. All rights reserved. 

quater conjugate(quater const& q)
{
	return quater(q.w, q.x*-1, q.y*-1, q.z*-1);
}


const dualQuaternion dualQuaternion::zero(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
const dualQuaternion dualQuaternion::identity(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

dualQuaternion const& dualQuaternion::operator=(matrix4 const& trans_mat)
{
	// Use MTransformationMatrix instead of matrix4 in case the transform
	// contains scaling. Otherwise the rotation quaternion will not be of unit length.
	//MTransformationMatrix trans_mat(inRigidTransform);

	quater rot ;
	rot.setRotation(trans_mat);
	vector3 pos		;
	pos.translation(trans_mat);
	
	dualQuaternion dq(pos, rot);
	mReal = dq.mReal;
	mDual = dq.mDual;

	return *this;
}

matrix4 dualQuaternion::asMatrix() const
{
	// Fill in rotation part
	matrix4 mat			;
	mat.setRotation(getRotation());
	
	// Fill in translation part
	vector3 translation = getTranslation();
	mat.setTranslation(translation);

	return mat;
}

matrix4 dualQuaternion::asMatrixFromUnit() const
{
	assert(isUnit());

	// Fill in rotation part
	matrix4 mat			;
	mat.setRotation(getRotation());
	
	// Fill in translation part
	vector3 translation = getTranslationFromUnit();
	mat.setTranslation(translation);
	
	return mat;
}

vector3 dualQuaternion::transformFromUnit(vector3 const& inPoint) const
{
	// Instead of getting the transformation matrix, it can be computed directly:
	// P' = (Qr.P.Qr~)/S + (Qd.Qr~ - Qr.Qd~))/S
	// Where:
	//   Qr is real quaternion part
	//   Qd is dual quaternion part
	//   ~ means conjugate
	//   S is dot(Qr,Qr). This is 1.0 for unit dual quaternions.
	// The first part of the equation is the "ordinary" rotate-vector-by-quaternion
	// equation. The second part is the same as calling dualQuaternion::getTranslation().
	//

	// Here is a sample implementation using vector math, as a possible implementation
	// for use with SIMD instructions. It's here for demonstration purposes, not
	// a truly optimized version.

	assert(isUnit());

	quater const& q = mReal;
	vector3 p(inPoint);
	vector3 qxyz(mReal.x,mReal.y,mReal.z);

	// Transforming a vector by a unit quaternion is done by: P' = Q.P.Q~
	// This boils down to vector math as: P' = P + 2*(Q.xyz ?(Q.xyz ?P + Q.w*P))
	// Do rotation of <inPoint> using vector math
	vector3 qxyzcrossp_plus_qwdotp = (qxyz.cross(p)) + q.w*p;
	p+=2.0*(qxyz.cross(qxyzcrossp_plus_qwdotp));

	// Add translation part

	// Vector math for quaternion multiply for two quaternions P and Q where
	// P(w,<xyz>) and Q(w,<xyz>), w is scalar part and xyz is vector part. Then:
	// P*Q = (P.w*Q.w + dot(P.xyz, Q.xyz), < P.w*Q.xyz + Q.w*P.xyz + P.xyz ?Q.xyz >)
	//
	// For the translation, we need 2.0*mDual*mReal.conjugate() and only the vector part
	// =>  2.0 * (mDual.w*-mReal.xyz + mReal.w*mDual.xyz + mDual.xyz ?-mReal.xyz)
	// => -2.0 * (mDual.w* mReal.xyz - mReal.w*mDual.xyz - mDual.xyz ? mReal.xyz)
	// ... to avoid taking the conjugate
	vector3 dxyz(mDual.x,mDual.y,mDual.z);
	vector3 t = dxyz.cross(qxyz);
	t+= mDual.w*qxyz;
	t+=-mReal.w*dxyz;
	p+=-2.0*t;

	return p;
}

dualQuaternion const& dualQuaternion::invertIt()
{
	double sqr_len_0 = mReal% mReal;
	double sqr_len_e = 2.0*mReal% mDual;

	if (sqr_len_0>0.0)
	{
		double inv_sqr_len_0 = 1.0/sqr_len_0;
		double inv_sqr_len_e = -sqr_len_e/(sqr_len_0*sqr_len_0);

		dualQuaternion conj = conjugate();
		mReal = inv_sqr_len_0*conj.mReal;
		mDual = inv_sqr_len_0*conj.mDual + inv_sqr_len_e*conj.mReal;
	}
	else
		(*this) = dualQuaternion::zero;

	return *this;
}

dualQuaternion const& dualQuaternion::setFromScrew(double inAngle, double inPitch, vector3 const& inDir, vector3 const& inMoment)
{
	double sin_half_angle = sin(inAngle*0.5);
	double cos_half_angle = cos(inAngle*0.5);
	
	mReal.w = cos_half_angle;
	mReal.x = sin_half_angle*inDir.x;
	mReal.y = sin_half_angle*inDir.y;
	mReal.z = sin_half_angle*inDir.z;

	mDual.w = -inPitch*sin_half_angle*0.5;
	mDual.x = sin_half_angle*inMoment.x + 0.5*inPitch*cos_half_angle*inDir.x;
	mDual.y = sin_half_angle*inMoment.y + 0.5*inPitch*cos_half_angle*inDir.y;
	mDual.z = sin_half_angle*inMoment.z + 0.5*inPitch*cos_half_angle*inDir.z;

	return *this;
}

void dualQuaternion::toScrew(double& outAngle, double& outPitch, vector3 &outDir, vector3 &outMoment) const
{
	assert(isUnit());

	// See if it's a pure translation:
	if (isPureTranslation())
	{
		outAngle			= 0.0;
		outDir.x			= mDual.x; 
		outDir.y			= mDual.y; 
		outDir.z			= mDual.z;

		double dir_sq_len	= outDir.x*outDir.x + outDir.y*outDir.y + outDir.z*outDir.z;
		
		// If a translation is nonzero, normalize is
		// else leave <outDir> zero vector (no motion at all)
		if (dir_sq_len>1e-6)
		{
			double dir_len	= sqrt(dir_sq_len);
			outPitch		= 2.0*dir_len;
			outDir			/= dir_len;
		}
		else
			outPitch		= 0.0;
		
		// Moment can be arbitrary
		outMoment			= vector3(0,0,0);
	}
	else
	{ 
		// Rigid transformation with a nonzero rotation
		outAngle			= 2.0*acos(mReal.w);

		double s			= mReal.x*mReal.x + mReal.y*mReal.y + mReal.z*mReal.z;
		if (s<1e-6)
		{
			outDir			= vector3(0,0,0);
			outPitch		= 0.0;
			outMoment		= vector3(0,0,0);
		}
		else
		{
			double oos		= 1.0/sqrt(s);
			outDir.x		= mReal.x * oos;
			outDir.y		= mReal.y * oos;
			outDir.z		= mReal.z * oos;

			outPitch		= -2.0*mDual.w*oos;

			outMoment.x 	= mDual.x; 
			outMoment.y 	= mDual.y; 
			outMoment.z 	= mDual.z;

			outMoment		= (outMoment - outDir*outPitch*mReal.w*0.5) * oos;
		}
	}
}

dualQuaternion dualQuaternion::log() const
{
	double angle, pitch;
	vector3 direction, moment;
	toScrew(angle, pitch, direction, moment);

	dualQuaternion res;
	res.mReal.x = direction.x*angle*0.5;
	res.mReal.y = direction.y*angle*0.5;
	res.mReal.z = direction.z*angle*0.5;
	res.mReal.w = 0.0;

	res.mDual.x = moment.x*angle*0.5 + direction.x*pitch*0.5;
	res.mDual.y = moment.y*angle*0.5 + direction.y*pitch*0.5;
	res.mDual.z = moment.z*angle*0.5 + direction.z*pitch*0.5;
	res.mDual.w = 0.0;

	return res;
}

dualQuaternion dualQuaternion::exp() const
{
	dualQuaternion res;
	vector3 n(mReal.x, mReal.y, mReal.z);

	double half_angle = n.length();

	// Pure translation?
	if (half_angle<1e-5)
		return dualQuaternion(quater(1,0,0,0), mDual);

	// Get normalized dir
	vector3 dir = (1.0/half_angle) * n;

	vector3 d(mDual.x, mDual.y, mDual.z);
	double half_pitch = d%dir;
	vector3 mom = (d - dir*half_pitch) / half_angle;

	return res.setFromScrew(half_angle*2.0, half_pitch*2.0, dir, mom);
}



//
// Do screw linear interpolation (the "slerp" for dual quaternions) for two unit dual quaternions
//
dualQuaternion dualQuaternion::sScLERP(double inT, dualQuaternion const& inFrom, dualQuaternion const& inTo)
{
	assert(0.0<=inT && inT<=1.0);	
	assert(inFrom.isUnit() && inTo.isUnit());

	// Make sure dot product is >= 0
	double quat_dot = inFrom.dotReal(inTo);
	dualQuaternion to_sign_corrected = (quat_dot>=0.0) ? inTo : -inTo;

	dualQuaternion dif_dq = inFrom.inverse() * to_sign_corrected;
	
	double  angle, pitch;
	vector3 direction, moment;
	dif_dq.toScrew(angle, pitch, direction, moment);

	angle *= inT; 
	pitch *= inT;
	dif_dq.setFromScrew(angle, pitch, direction, moment);

	return inFrom * dif_dq;
}



//
// Do dual quaternion linear interpolation. Result is normalized afterwards.
//
dualQuaternion dualQuaternion::sDLB(int inCount, const double inWeightList[], const dualQuaternion inDualQuatList[])
{
	assert(inCount>0);

	// Find max weight index for pivoting to that quaternion, so shortest arc is taken
	int pivot_idx = 0;
	for (int n=1; n<inCount; n++)
		if (inWeightList[pivot_idx] < inWeightList[n])
			pivot_idx = n;
	dualQuaternion const& dq_pivot = inDualQuatList[pivot_idx];

	dualQuaternion res(dualQuaternion::zero);
	for (int n=0; n<inCount; n++)
	{
		dualQuaternion const& dq = inDualQuatList[n];
		double weight = inWeightList[n];

		// Make sure dot product is >= 0
		if (dq.dotReal(dq_pivot)<0.0)
			weight = -weight;

		res += weight*dq;
	}

	res.normalizeIt();

	return res;
}



//
// Do dual quaternion iterative intrinsic interpolation up to given precision. Result is always normalized.
//
dualQuaternion dualQuaternion::sDIB(int inCount, const double inWeightList[], const dualQuaternion inDualQuatList[], double inPrecision)
{
	assert(inCount>0);
	dualQuaternion b = sDLB(inCount, inWeightList, inDualQuatList);

	// Find max weight index for pivoting to that quaternion, so shortest arc is taken
	int pivot_idx = 0;
	for (int n=1; n<inCount; n++)
		if (inWeightList[pivot_idx] < inWeightList[n])
			pivot_idx = n;
	dualQuaternion const& dq_pivot = inDualQuatList[pivot_idx];

	// Iteratively refine
	enum { MAX_DIB_ITERS = 20 };
	for (int iter=0; iter<MAX_DIB_ITERS; ++iter)
	{
		const dualQuaternion inv_b = b.inverse();
		dualQuaternion x(dualQuaternion::zero);
		for (int i=0; i<inCount; i++)
		{
			dualQuaternion dq		= inDualQuatList[i];
			double weight			= inWeightList[i];

			// Make sure dot product is >= 0
			if (dq.dotReal(dq_pivot)<0.0)
				dq.negateIt();

			dualQuaternion xi		= inv_b * dq;
			x += weight*xi.log();
		}

		double norm = x.dot(x);

		b = b * x.exp();

		if (norm < inPrecision) 
			return b;				
	}

	// Failed to converge. At least normalize.
	b.normalizeIt();

	return b;
}

