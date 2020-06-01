#ifndef PROFILE_H_
#define PROFILE_H_

#define PROFILE
#ifdef PROFILE
#include "../../taesooLib/BaseLib/utility/QPerformanceTimer.h"
extern QPerformanceTimerCount2 gSplineAccTimer; // spline_acc_constraint.h
extern QPerformanceTimerCount2 gDynTimer; // relaxed_dynamic_constraint.h
extern QPerformanceTimerCount2 gJacTimer; // leaves.cc
extern QPerformanceTimerCount2 gForceCTimer; // force_constraint.h
extern QPerformanceTimerCount2 gECCTimer; // euler_converter.h
extern QPerformanceTimerCount2 gReallocTimer;
extern QPerformanceTimerCount2 gCCTimer;
#endif

#define USE_OPTIMIZED_CODE // no improvement -.-
#include <iostream>
typedef Eigen::Matrix<bool,3,1> bool3;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor>  Jac_;
inline static void argMin(int& minc, int&  argc, Jac_::InnerIterator * itp[3], Eigen::Vector3d&v)
{
	int argminc;
	minc=INT_MAX;
	argc=0;
	for(int i=0; i<3; i++)
	{
		if(*itp[i])
		{
			argc++;
			int c=(*(itp[i])).col();
			if(c<minc)
			{
				minc=c;
				argminc=i;
			}
		}
	}
	for(int i=0; i<3; i++)
	{
		if(*itp[i])
		{
			if((*(itp[i])).col()==minc)
			{
				v(i)=((*itp[i])).value();
				++(*itp[i]);
			}
			else
				v(i)=0.0;
		}
		else
			v(i)=0.0;
	}
}
typedef Eigen::SparseMatrix<double, Eigen::RowMajor>  Jac_;
inline static void argMin(int& minc, int&  argc, Jac_::InnerIterator * itp[3], bool3& bb, Eigen::Vector3d&v)
{
	int argminc;
	minc=INT_MAX;
	argc=0;
	for(int i=0; i<3; i++)
	{
		if(*itp[i])
		{
			argc++;
			int c=(*(itp[i])).col();
			if(c<minc)
			{
				minc=c;
				argminc=i;
			}
		}
	}
	for(int i=0; i<3; i++)
	{
		if(*itp[i])
		{
			if((*(itp[i])).col()==minc)
			{
				v(i)=((*itp[i])).value();
				bb(i)=false;
				++(*itp[i]);
			}
			else
			{
				v(i)=0.0;
				bb(i)=true;
			}
		}
		else
		{
			v(i)=0.0;
			bb(i)=true;
		}
	}
}

inline Eigen::Matrix3d Cross_(const Eigen::Vector3d& t)
{
	Eigen::Matrix3d t_hat;
	t_hat << 0, -t(2), t(1),
		  t(2), 0, -t(0),
		  -t(1), t(0), 0;
	return t_hat;
}
//
// assumes a(0,0) is always zero
inline Jac_ multJac_zero00(Eigen::Matrix3d const& a, const Jac_& b)
{
	Jac_ out(b.rows(), b.cols());
	out.reserve(b.nonZeros()*3);// worst case 
#ifdef _DEBUG
	assert(b.outerSize()==3);
#endif
	Jac_::InnerIterator itx(b, 0);
	Jac_::InnerIterator ity(b, 1);
	Jac_::InnerIterator itz(b, 2);
	Jac_::InnerIterator * itp[3];
	itp[0]=&itx;
	itp[1]=&ity;
	itp[2]=&itz;

	int curCol=0;
	int argc=3;
	Eigen::Vector3d v;
	Eigen::Vector3d o;
	bool3 bb;
	while(1)
	{
		argMin(curCol, argc,itp, bb, v);
		if(argc==0) break;
		o=a*v;

		if(!(bb(1)&&bb(2))) out.insert(0, curCol)=o(0);
		out.insert(1, curCol)=o(1);
		out.insert(2, curCol)=o(2);
	}
	return out;
}
template <bool prune>
inline Jac_ multJac(Eigen::Matrix3d const& a, const Jac_& b)
{
#ifdef PROFILE
	gJacTimer.start();
#endif
	Jac_ out(b.rows(), b.cols());
	out.reserve(b.nonZeros()*3);// worst case 
#ifdef _DEBUG
	assert(b.outerSize()==3);
#endif
	Jac_::InnerIterator itx(b, 0);
	Jac_::InnerIterator ity(b, 1);
	Jac_::InnerIterator itz(b, 2);
	Jac_::InnerIterator * itp[3];
	itp[0]=&itx;
	itp[1]=&ity;
	itp[2]=&itz;

	int curCol=0;
	int argc=3;
	Eigen::Vector3d v;
	Eigen::Vector3d o;
	while(1)
	{
		argMin(curCol, argc,itp, v);
		if(argc==0) break;
		o=a*v;
		if(prune){
			if(o(0)!=0.0) out.insert(0, curCol)=o(0);
			if(o(1)!=0.0) out.insert(1, curCol)=o(1);
			if(o(2)!=0.0) out.insert(2, curCol)=o(2);
		}
		else {
			out.insert(0, curCol)=o(0);
			out.insert(1, curCol)=o(1);
			out.insert(2, curCol)=o(2);
		}
	}
#ifdef PROFILE
	gJacTimer.pause();
#endif
	return out;
}
template <bool prune>
inline Jac_ multCross(Eigen::Vector3d const& vv, const Jac_& b)
{
	Eigen::Matrix3d a=Cross_(vv);
	Jac_ out(b.rows(), b.cols());
	out.reserve(b.nonZeros()*3);// worst case 
#ifdef _DEBUG
	assert(b.outerSize()==3);
#endif
	Jac_::InnerIterator itx(b, 0);
	Jac_::InnerIterator ity(b, 1);
	Jac_::InnerIterator itz(b, 2);
	Jac_::InnerIterator * itp[3];
	itp[0]=&itx;
	itp[1]=&ity;
	itp[2]=&itz;

	int curCol=0;
	int argc=3;
	Eigen::Vector3d v;
	Eigen::Vector3d o;
	bool3 bb;
	while(1)
	{
		argMin(curCol, argc,itp, bb, v);
		if(argc==0) break;
		o=a*v;
#ifdef _DEBUG
		if(bb(1)&&bb(2)) assert(o(0)==0.0);
		if(bb(0)&&bb(2)) assert(o(1)==0.0);
		if(bb(0)&&bb(1)) assert(o(2)==0.0);
#endif
		if (prune)
		{
			if(!(bb(1)&&bb(2))) out.insert(0, curCol)=o(0);
			if(!(bb(0)&&bb(2))) out.insert(1, curCol)=o(1);
			if(!(bb(0)&&bb(1))) out.insert(2, curCol)=o(2);
		}
		else
		{
			out.insert(0, curCol)=o(0);
			out.insert(1, curCol)=o(1);
			out.insert(2, curCol)=o(2);
		}
	}
	return out;
}
inline void printJac(Jac_ const&jac_force)
{
  std::cout<<jac_force<<std::endl;
  for(int k=0; k<jac_force.outerSize(); k++)
  {
	  std::cout<< " r: "<< k;
	  for(Jac_::InnerIterator it(jac_force, k); it; ++it)
	  {
		  std::cout<< " c: " << it.col();
	  }
	  std::cout<<std::endl;
  }
}
class JacobianInvMap
{
	public:
		using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
		JacobianInvMap(){}
		Eigen::MatrixXi invmap;
		void initFrom(Jacobian const& j)
		{
			const double* v= j.valuePtr();      // Pointer to the values
			const int *ii =j.innerIndexPtr();  // Pointer to the indices.
			const int *oi =j.outerIndexPtr(); // Pointer to the beginning of each 
			invmap.resize(j.rows(), j.cols());
			invmap.array()=-1;
			int c=0;
			for(int i=0; i<j.rows(); i++)
			{
				for(Jacobian::InnerIterator it(j, i); it; ++it)
				{
					assert(it.col()==ii[c]);
					assert(it.row()==i);
					assert(&it.value()==&v[c]);
					invmap(it.row(), it.col())=c; 
					c++;
				}
			}
		}

		inline void setLValue(Jacobian& out) const
		{
			_v=out.valuePtr();
#ifdef _DEBUG
			_ii =out.innerIndexPtr();  // Pointer to the indices.
			_oi =out.outerIndexPtr(); // Pointer to the beginning of each 
			_nrow=out.rows();
#endif
		}
		inline double& coeffRef(int row, int col) const
		{
#ifdef _DEBUG
			int c=invmap(row, col);
			assert(_ii[c]==col);
			assert(c>=_oi[row] && (row==_nrow-1 || c< _oi[row+1]));
#endif
			return _v[invmap(row, col)];
		}
	private:
		mutable double* _v;
#ifdef _DEBUG
		mutable int *_ii;
		mutable int *_oi;
		mutable int _nrow;
#endif
};


#endif // PROFILE_H_
