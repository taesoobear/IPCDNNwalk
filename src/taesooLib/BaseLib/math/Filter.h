#ifndef _FILTER_H_
#define _FILTER_H_

#if _MSC_VER>1000
#pragma once
#endif
#include "mathclass.h"

class Motion;

/**
 * filtering관련.
 */
class Filter
{
public:
	Filter();
	virtual ~Filter();

	// shortcuts
	static void gaussFilter(int kernelsize, matrixn& inout);
	static void gaussFilter(int kernelSize, vectorn& inout);
	static void medianFilter(int kernelSize, matrixn& inout);
	static void medianFilter(int kernelSize, vectorn& inout);


	// implementations
	static int CalcKernelSize(float time, float fFrameTime) { float half_frames=time/(2*fFrameTime); return (ROUND(half_frames))*2+1;};
	static int KernelSize(Motion const& in, float time);

	static void GetBoxFilter(int kernelsize, vectorn &kernel);
	static void GetBlurFilter(int kernelsize, vectorn &kernel);
	static void GetGaussFilter(int kernelsize, vectorn &kernel);
	static void GetTransitionKernel(int kernelsize, vectorn& kernel);	//!< 0~1 smooth transition

	//! Varying kernel size
	static void LTIFilter(int numIter, float fFrameTime, const vectorn& kernel_size, const vectorn& aInput, vectorn& aOutput);// second

	//! Filter::GetBlurFilter로 얻은 마스크를 kernel에 넣어주면 블러가 된다.
	static void LTIFilter(int numIter, const vectorn& kernel, vectorn& inout);
	static void LTIFilter(int numIter, const vectorn& kernel, const vectorn& in, vectorn& out);
	//! Filter::GetBlurFilter로 얻은 마스크를 kernel에 넣어주면 블러가 된다.
	static void LTIFilter(int numIter, const vectorn& kernel, const matrixn& in, matrixn& out);
	static void LTIFilter(int numIter, const vectorn& kernel, matrixn& inout)
	{	matrixn sourceCopy;	sourceCopy.assign(inout); LTIFilter(numIter, kernel, sourceCopy, inout);}

	static void LTIFilterQuat(int numIter, const vectorn& kernel, const matrixn& in, matrixn& out);
	static void LTIFilterQuat(int numIter, const vectorn& kernel, matrixn& inout)
	{	matrixn sourceCopy;	sourceCopy.assign(inout); LTIFilterQuat(numIter, kernel, sourceCopy, inout);}

#ifdef DIRECT3D_VERSION
	//! 모든 종류의 filtering을 하기전에 모든 quaternion은 반드시 align되어 있어야 한다
	static void AlignUnitQuaternions( int numQuat, D3DXQUATERNION* aQuat);
	//! Filter::GetBlurFilter로 얻은 마스크를 kernel에 넣어주면 블러가 된다.
	static void LTIFilter(int numIter, const vectorn& kernel, int numVec3, D3DXVECTOR3* aVec3);
	//! Filter::GetBlurFilter로 얻은 마스크를 kernel에 넣어주면 블러가 된다.
	static void LTIFilter(int numIter, const vectorn& kernel, int numQuat, D3DXQUATERNION* aQuat);
	static void Smoothing(int numIter, int numVec3, D3DXVECTOR3* aVec3);
	static void Smoothing(int numIter, int numQuat, D3DXQUATERNION* aQuat);
#endif

private:
	static void GetBoxFilter(int kernelsize, float* ai_array);
	static void GetBlurFilter(int kernelsize, float* ai_array);
	//! Filter::GetBlurFilter로 얻은 마스크를 ai_array에 넣어주면 블러가 된다.
	static void LTIFilter(int numIter, const vectorn& kernel, int numFloat, m_real* aInFloat, m_real* aOutFloat);
};

#endif
