// Filter.cpp: implementation of the Filter class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Filter.h"
#include <math.h>
#include "../math/Operator.h"
#include "../motion/Motion.h"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Filter::Filter()
{
}

Filter::~Filter()
{

}

int Filter::KernelSize(Motion const& in, float time) { return CalcKernelSize(time, in.frameTime());}
void Filter::gaussFilter(int kernelSize, matrixn& inout)
{
	if(kernelSize%2==0) kernelSize=kernelSize/2*2+1;
	vectorn kernel;
	GetGaussFilter(kernelSize, kernel);
	LTIFilter(1,kernel,inout);
}

void Filter::gaussFilter(int kernelSize, vectorn& inout)
{
	if(kernelSize%2==0) kernelSize=kernelSize/2*2+1;
	vectorn kernel;
	GetGaussFilter(kernelSize, kernel);
	LTIFilter(1,kernel,inout);
}


#ifdef DIRECT3D_VERSION
void Filter::AlignUnitQuaternions( int numQuat, D3DXQUATERNION* aQuat)
{
    for( int i=1; i<numQuat; i++ )
        if ( D3DXQuaternionDot(&aQuat[i-1],&aQuat[i])<0 ) aQuat[i] = -aQuat[i];
}

void Filter::Smoothing(int numIter, int numVec3, D3DXVECTOR3* aVec3)
{
	D3DXVECTOR3* sourceCopy;
	sourceCopy = new D3DXVECTOR3[numVec3];

	for(int i=0; i<numVec3; i++)
		sourceCopy[i] = aVec3[i];

	for( int j=0; j<numIter; j++ )
    {
        aVec3[1] = (2*sourceCopy[0] + 11*sourceCopy[1] + 4*sourceCopy[2] - sourceCopy[3])/16.0;

        for( int i=2; i<numVec3-2; i++ )
            aVec3[i] = (- sourceCopy[i-2] + 4*sourceCopy[i-1]
					   + 10*sourceCopy[i] + 4*sourceCopy[i+1] - sourceCopy[i+2])/16.0;

        aVec3[numVec3-2] = (2*sourceCopy[numVec3-1] + 11*sourceCopy[numVec3-2] + 4*sourceCopy[numVec3-3] - sourceCopy[numVec3-4])/16.0;
    }
	delete[] sourceCopy;
}
#endif
double FactorialPerFactorial(int a, int b)
{
	// a!/ b!를 계산한다.
	ASSERT(a>=b);
	if(a==b) return 1;
	double result=1.0;
	for(int i=a; i>b; i--)
	{
		result*=(double)i;
	}
	return result;
}
double Factorial(int a)
{
	if(a==0) return 1.0;
	double result=1.0;
	for(double i=1; i<=a; i++)
		result*=(double)i;
	return result;
}
float Sum(int from , int to, m_real* array)
{
	m_real sum=0;
	for(int i=from; i<=to; i++)
	{
		sum+=array[i];
	}

	return sum;
}

float Sum(int from , int to, const vectorn& array)
{
	m_real sum=0;
	for(int i=from; i<=to; i++)
	{
		sum+=array[i];
	}

	return sum;
}

//원래 -k부터 시작하는 인덱스를 0부터 시작하도록 바꿔주는 매크로 함수
#define TO_ARRAY_INDEX(x)	((x)+k)

void Filter::GetBoxFilter(int kernelsize, vectorn &ai_array)
{
	ai_array.setSize(kernelsize);
	// Make Binomial coefficients array
	ASSERT(kernelsize%2==1);

	for(int i=0; i<kernelsize; i++)
	{
		ai_array[i]=1/(float)kernelsize;
	}

	TRACE("coef sum %f \n",Sum(0, kernelsize-1, ai_array));

}

void Filter::GetBlurFilter(int kernelsize, vectorn &ai_array)
{
	ai_array.setSize(kernelsize);

	// Make Binomial coefficients array
	ASSERT(kernelsize%2==1);
	int k=kernelsize/2;

	for(int i=0; i<=k; i++)
	{
		// 2k!/((k+i)!(k-i)!pow(2,2k))
		ai_array[TO_ARRAY_INDEX(i)]=(float)(((double)FactorialPerFactorial(2*k,k+i)/(double)Factorial(k-i))/pow(2.0,2.0*k));
	}

	// i가 -k부터0 까지는 대칭으로 넣어준다.
	for(int i=-k; i<0; i++)
		ai_array[TO_ARRAY_INDEX(i)]=ai_array[TO_ARRAY_INDEX(-1*i)];

	TRACE("coef sum %f \n",Sum(0, kernelsize-1, ai_array));
}


void Filter::medianFilter(int kernelSize, matrixn& inout)
{
	for(int i=0; i<inout.cols(); i++)
	{
		Filter::medianFilter(kernelSize, inout.column(i).lval());
	}

}
void Filter::medianFilter(int kernelSize, vectorn& inout)
{
	int hkernelSize=kernelSize/2;
	vectorn sample(hkernelSize*2);
	intvectorn sortedOrder;
	vectorn out(inout.size());
	for(int i=0; i<inout.size(); i++)
	{
		int s=i-hkernelSize;
		int e=i+hkernelSize;

		if(s<0)
		{
			sample=inout.range(0, hkernelSize*2);
		}
		else if(e>inout.size())
		{
			sample=inout.range(inout.size()-hkernelSize*2, inout.size());
		}
		else
		{
			sample=inout.range(s,e);
		}
		sortedOrder.sortedOrder(sample);	
		out[i]=sample[sortedOrder[(e-s)/2]];
	}
	inout=out;
}
void Filter::GetGaussFilter(int kernelsize, vectorn &ai_array)
{
	if(kernelsize%2==0) kernelsize=kernelsize/2*2+1;
	//GDTW: exp(-gamma*distance^2)
	ai_array.setSize(kernelsize);
	m_real gamma=1/(m_real) kernelsize;

	ASSERT(kernelsize%2==1);
	int k=kernelsize/2;

	for(int i=0; i<=k; i++)
	{
		ai_array[TO_ARRAY_INDEX(i)]=exp(gamma*-1*i*i);
	}
	// i가 -k부터0 까지는 대칭으로 넣어준다.
	for(int i=-k; i<0; i++)
		ai_array[TO_ARRAY_INDEX(i)]=ai_array[TO_ARRAY_INDEX(-1*i)];

	ai_array/=ai_array.sum();
	TRACE("coef sum %f \n",Sum(0, kernelsize-1, ai_array));
}

void Filter::LTIFilter(int numIter, const vectorn& kernel, vectorn& aFloat)
{
	vectorn sourceCopy(aFloat);
	LTIFilter(numIter, kernel, aFloat.size(), sourceCopy.dataPtr(), aFloat.dataPtr());
}

void Filter::LTIFilter(int numIter, const vectorn& kernel, const vectorn& in, vectorn& out)
{
	out.setSize(in.size());
	LTIFilter(numIter, kernel, in.size(), in.dataPtr(), out.dataPtr());
}


void Filter::LTIFilter(int numIter, float fFrameTime, const vectorn& kernel_size, const vectorn& aInput, vectorn& aOutput)
{
	intvectorn kernelSize;
	kernelSize.setSize(kernel_size.size());

	int upperBoundKernel=CalcKernelSize(2.f, fFrameTime);
	for(int i=0; i<kernelSize.size(); i++)
	{
		kernelSize[i]=MAX(0,CalcKernelSize(kernel_size[i], fFrameTime));
		if(kernelSize[i]>upperBoundKernel)
			kernelSize[i]=upperBoundKernel;
	}

	std::vector<vectorn> kernels;
	int maxKernel=kernelSize.maximum();
	kernels.resize(maxKernel+1);

	for(int i=1; i<=maxKernel; i+=2)
	{
		GetGaussFilter(i,kernels[i]);
	}


	int numFloat=aInput.size();
	aOutput.setSize(aInput.size());

	for(int j=0; j<numIter; j++)
	{
		for(int  i=0; i<numFloat; i++ )
		{

			if(kernelSize[i]==0)
				aOutput[i]=aInput[i];
			else
			{
				vectorn& ai_array=kernels[kernelSize[i]];
				int k=ai_array.size()/2;

				aOutput[i]=0;

				for(int m=-1*k; m<=k; m++)
				{
					if(i+m<0)
						aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[0];
					else if(i+m>=aInput.size())
						aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[aInput.size()-1];
					else
						aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[i+m];
				}
			}
		}
	}
}


void Filter::LTIFilter(int numIter, const vectorn& ai_array, int numFloat, m_real * aInput,m_real* aOutput)
{
	Msg::verify(ai_array.size()<numFloat, "LTIFilter:: kernel is too large!");
	int k=ai_array.size()/2;
	for(int j=0; j<numIter; j++)
	{
		// 앞부분은 첫 포스쳐를 반복해 넣는다..
		for( int i=0; i<k; i++)
		{
			aOutput[i]=0;

			for(int m=-1*k; m<=k ;m++)
			{
				if(i+m<0)
					aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[0];
				else
					aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[i+m];
			}
		}

		// 중간부분은 그대로 컨볼루션
		for( int i=k; i<numFloat-k; i++ )
		{
			aOutput[i]=0;

			for(int m=-1*k; m<=k; m++)
				aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[i+m];
		}
		// 뒷부분은 마지막 포스쳐를 반복해 넣는다.
		for( int i=numFloat-k; i<numFloat; i++)
		{
			aOutput[i]=0;

			for(int m=-1*k; m<=k ;m++)
			{
				if(i+m<numFloat)
					aOutput[i] += ai_array[TO_ARRAY_INDEX(m)]*aInput[i+m];
				else
					aOutput[i]+=ai_array[TO_ARRAY_INDEX(m)]*aInput[numFloat-1];
			}
		}
	}
}

//#ifdef DIRECT3D_VERSION
//void Filter::LTIFilter(int numIter, const vectorn& ai_array, int numVec3, D3DXVECTOR3* aVec3)
//{
//	D3DXVECTOR3* sourceCopy;
//	sourceCopy=new D3DXVECTOR3[numVec3];
//
//	for(int i=0; i<numVec3; i++)
//		sourceCopy[i]=aVec3[i];
//
//	int k=ai_array.size()/2;
//	for(int j=0; j<numIter; j++)
//	{
//		// 앞부분은 첫 포스쳐를 반복해 넣는다..
//		for( int i=0; i<k; i++)
//		{
//			aVec3[i].x=0; aVec3[i].y=0; aVec3[i].z=0;
//
//			for(int m=-1*k; m<=k ;m++)
//			{
//				if(i+m<0)
//					aVec3[i] += ai_array[TO_ARRAY_INDEX(m)]*sourceCopy[0];
//				else
//					aVec3[i] += ai_array[TO_ARRAY_INDEX(m)]*sourceCopy[i+m];
//			}
//		}
//
//		// 중간부분은 그대로 컨볼루션
//		for( i=k; i<numVec3-k; i++ )
//		{
//			aVec3[i].x=0;	aVec3[i].y=0;	aVec3[i].z=0;
//
//			for(int m=-1*k; m<=k; m++)
//				aVec3[i] += ai_array[TO_ARRAY_INDEX(m)]*sourceCopy[i+m];
//		}
//		// 뒷부분은 마지막 포스쳐를 반복해 넣는다.
//		for( i=numVec3-k; i<numVec3; i++)
//		{
//			aVec3[i].x=0; aVec3[i].y=0; aVec3[i].z=0;
//
//			for(int m=-1*k; m<=k ;m++)
//			{
//				if(i+m<numVec3)
//					aVec3[i] += ai_array[TO_ARRAY_INDEX(m)]*sourceCopy[i+m];
//				else
//					aVec3[i]+=ai_array[TO_ARRAY_INDEX(m)]*sourceCopy[numVec3-1];
//			}
//		}
//	}
//	delete[] sourceCopy;
//}
//
//
//void Filter::LTIFilter(int numIter, const vectorn& ai_array, int numQuat, D3DXQUATERNION* aQuat)
//{
//	float* bi_array=new float[ai_array.size()-1];
//
//	int k=ai_array.size()/2;
//
//	// calculate b_sub_i .. 제이형 논문 참고. bm=SUM a_sub_i 형태로 정의됨.
//	for(int m=-k; m<0; m++)
//		bi_array[TO_ARRAY_INDEX(m)]=-1*Sum(TO_ARRAY_INDEX(-k), TO_ARRAY_INDEX(m), ai_array);
//
//	for(m=0; m<=k-1; m++)
//		bi_array[TO_ARRAY_INDEX(m)]=Sum(TO_ARRAY_INDEX(m+1), TO_ARRAY_INDEX(k), ai_array);
//
//	D3DXQUATERNION* w;	// (0, theta*v)
//	w = new D3DXQUATERNION[numQuat-1];
//
//	D3DXQUATERNION qc, sum, temp;
//	for(int j=0;j<numIter;j++)
//	{
//
//		for(int i=0; i<numQuat-1; i++ )
//		{
//			D3DXQuaternionInverse(&qc, &aQuat[i]);
//			D3DXQuaternionMultiply(&temp, &qc, &aQuat[i+1]);
//			D3DXQuaternionLn(&w[i], &temp);
//		}
//
//		// 앞부분은 첫 포스쳐를 반복해 넣는다.. 이경우 w가 0벡터가 되므로 sum에 더하지 않으면 된다.
//		for( i=0; i<k; i++)
//		{
//			sum.w=0;	sum.x=0;	sum.y=0;	sum.z=0;
//
//			for(m=-1*k; m<k; m++)
//			{
//				if(i+m>=0)
//					sum+=bi_array[TO_ARRAY_INDEX(m)]*w[i+m];
//			}
//
//			D3DXQuaternionExp(&qc, &sum);
//			D3DXQuaternionMultiply(&aQuat[i], &aQuat[i], &qc);
//
//		}
//
//		// 중간 부분은 마스크를 고대로 컨볼루션
//		for( i=k; i<numQuat-k; i++ )
//		{
//			sum.w=0;	sum.x=0;	sum.y=0;	sum.z=0;
//
//			for(m=-1*k; m<k; m++)
//				sum+=bi_array[TO_ARRAY_INDEX(m)]*w[i+m];
//
//			D3DXQuaternionExp(&qc, &sum);
//			D3DXQuaternionMultiply(&aQuat[i], &aQuat[i], &qc);
//		}
//
//		// 뒷부분은 마지막 포스쳐를 반복해 넣는다..
//		for( i=numQuat-k; i<numQuat; i++)
//		{
//			sum.w=0;	sum.x=0;	sum.y=0;	sum.z=0;
//
//			for(m=-1*k; m<k; m++)
//			{
//				if(i+m<numQuat-1)
//					sum+=bi_array[TO_ARRAY_INDEX(m)]*w[i+m];
//			}
//
//			D3DXQuaternionExp(&qc, &sum);
//			D3DXQuaternionMultiply(&aQuat[i], &aQuat[i], &qc);
//		}
//
//		for( i=0; i<numQuat; i++)
//			D3DXQuaternionNormalize(&aQuat[i], &aQuat[i]);
//	}
//}
//
//void Filter::Smoothing(int numIter, int numQuat, D3DXQUATERNION* aQuat)
//{
//	float lambda=1.0f;
//	D3DXQUATERNION qc;
//	D3DXQUATERNION qn;
//	D3DXQUATERNION temp;
//	D3DXQUATERNION* w;	// (0, theta*v)
//	int i, k;
//
//	w = new D3DXQUATERNION[numQuat-1];
//
//	for(k=0;k<numIter;k++)
//	{
//
//		for( i=0; i<numQuat-1; i++ )
//		{
//			D3DXQuaternionInverse(&qc, &aQuat[i]);
//			D3DXQuaternionMultiply(&temp, &qc, &aQuat[i+1]);
//			D3DXQuaternionLn(&w[i], &temp);
//		}
//
//		D3DXQuaternionExp(&qc, &((lambda/24)*(3*w[0]-w[1])));
//		D3DXQuaternionMultiply(&aQuat[0], &aQuat[0], &qc);
//
//		D3DXQuaternionExp(&qc, &((lambda/24)*(3*w[0]+3*w[1]-w[2])));
//		D3DXQuaternionMultiply(&aQuat[1], &aQuat[1], &qc);
//
//		for( i=2; i<numQuat-2; i++ )
//		{
//			D3DXQuaternionExp(&qc, &((lambda/24)*(w[i-2]-3*w[i-1]+3*w[i]-w[i+1])));
//			D3DXQuaternionMultiply(&aQuat[i], &aQuat[i], &qc);
//		}
//
//		D3DXQuaternionExp(&qc, &((lambda/24)*(w[i-2]-3*w[i-1]+3*w[i])));
//		D3DXQuaternionMultiply(&aQuat[i], &aQuat[i], &qc);
//
//		i++;
//		D3DXQuaternionExp(&qc, &((lambda/24)*(w[i-2]-3*w[i-1])));
//		D3DXQuaternionMultiply(&aQuat[i], &aQuat[i], &qc);
//
//
//		for( i=0; i<numQuat; i++)
//			D3DXQuaternionNormalize(&aQuat[i], &aQuat[i]);
//	}
//
//	delete[] w;
//
//}
//#endif
#include "mathclass.h"

void Filter::LTIFilter(int numIter, const vectorn& ai_array, const matrixn& sourceCopy, matrixn& aVecN)
{
	/*
	int numVector=aVecN.rows();

	int k=ai_array.size()/2;
	for(int j=0; j<numIter; j++)
	{
		// 앞부분은 첫 포스쳐를 반복해 넣는다..
		for( int i=0; i<k; i++)
		{
			aVecN.row(i).setAllValue(0);

			for(int m=-1*k; m<=k ;m++)
			{
				if(i+m<0)
					aVecN.row(i) += sourceCopy.row(0)*ai_array[TO_ARRAY_INDEX(m)];
				else
					aVecN.row(i) += sourceCopy.row(i+m)*ai_array[TO_ARRAY_INDEX(m)];
			}
		}

		// 중간부분은 그대로 컨볼루션
		for( int i=k; i<numVector-k; i++ )
		{
			aVecN.row(i).setAllValue(0);

			for(int m=-1*k; m<=k; m++)
				aVecN.row(i) += sourceCopy.row(i+m)*ai_array[TO_ARRAY_INDEX(m)];
		}
		// 뒷부분은 마지막 포스쳐를 반복해 넣는다.
		for(int i=numVector-k; i<numVector; i++)
		{
			aVecN.row(i).setAllValue(0);

			for(int m=-1*k; m<=k ;m++)
			{
				if(i+m<numVector)
					aVecN.row(i)+=sourceCopy.row(i+m)*ai_array[TO_ARRAY_INDEX(m)];
				else
					aVecN.row(i)+=sourceCopy.row(numVector-1)*ai_array[TO_ARRAY_INDEX(m)];
			}
		}
	}*/

	matrixn sourceTranspose;
	sourceTranspose.transpose(sourceCopy);
	matrixn aVecNTranspose;
	aVecNTranspose.setSameSize(sourceTranspose);

	for(int i=0; i<sourceTranspose.rows(); i++)
	{
	    vectornView src_row_i=sourceTranspose.row(i);
	    vectornView dst_row_i=aVecNTranspose.row(i);
		LTIFilter(numIter, ai_array, src_row_i, dst_row_i);
	}

	aVecN.transpose(aVecNTranspose);
}

void Filter::GetTransitionKernel(int kernelsize, vectorn& kernel)
{
	vectorn range(kernelsize);
	range.linspace(0,1);
	// y=-2/3x^3+2/3x^2
	for(int i=0; i<range.size(); i++)
		kernel[i]=sop::smoothTransition(kernel[i]);
}

void Filter::LTIFilterQuat(int numIter, const vectorn& ai_array, const matrixn& input, matrixn& out)
{
	vectorn bi_array;
	bi_array.setSize(ai_array.size()-1);

	int k=ai_array.size()/2;

	// calculate b_sub_i .. 제이형 논문 참고. bm=SUM a_sub_i 형태로 정의됨.
	for(int m=-k; m<0; m++)
		bi_array[TO_ARRAY_INDEX(m)]=-1*Sum(TO_ARRAY_INDEX(-k), TO_ARRAY_INDEX(m), ai_array);

	for(int m=0; m<=k-1; m++)
		bi_array[TO_ARRAY_INDEX(m)]=Sum(TO_ARRAY_INDEX(m+1), TO_ARRAY_INDEX(k), ai_array);

	int numQuat=input.rows();

	vector3* w;	// (0, theta*v)
	w = new vector3[numQuat-1];
	quater* in;
	in = new quater[numQuat];
	for(int i=0; i<numQuat; i++)
		in[i]=input.row(i).toQuater();

	// align quaternion

    for( int i=1; i<numQuat; i++ )
		if ( in[i-1]%in[i]<0 ) in[i].negate();

	vector3 sum;
	quater qc, temp;
	quater qout;
	for(int j=0;j<numIter;j++)
	{

		for(int i=0; i<numQuat-1; i++ )
		{
			qc.inverse(in[i]);
			temp.mult(qc, in[i+1]);
			w[i].ln(temp);
		}

		// 앞부분은 첫 포스쳐를 반복해 넣는다.. 이경우 w가 0벡터가 되므로 sum에 더하지 않으면 된다.
		for( int i=0; i<k; i++)
		{
			sum.x=0;	sum.y=0;	sum.z=0;

			for(int m=-1*k; m<k; m++)
			{
				if(i+m>=0)
					sum+=bi_array[TO_ARRAY_INDEX(m)]*w[i+m];
			}

			qc.exp(sum);
			qout.mult(in[i], qc);
			qout.normalize();
			out.row(i).assign(qout);
		}

		// 중간 부분은 마스크를 고대로 컨볼루션
		for( int i=k; i<numQuat-k; i++ )
		{
			sum.x=0;	sum.y=0;	sum.z=0;

			for(int m=-1*k; m<k; m++)
				sum+=bi_array[TO_ARRAY_INDEX(m)]*w[i+m];

			qc.exp(sum);
			qout.mult(in[i], qc);
			qout.normalize();
			out.row(i).assign(qout);
		}

		// 뒷부분은 마지막 포스쳐를 반복해 넣는다..
		for(int  i=numQuat-k; i<numQuat; i++)
		{
			sum.x=0;	sum.y=0;	sum.z=0;

			for(int m=-1*k; m<k; m++)
			{
				if(i+m<numQuat-1)
					sum+=bi_array[TO_ARRAY_INDEX(m)]*w[i+m];
			}

			qc.exp(sum);
			qout.mult(in[i], qc);
			qout.normalize();
			out.row(i).assign(qout);
		}
	}
	delete[] w;
	delete[] in;
}

