#ifndef EIGEN_SUPPORT_H_
#define EIGEN_SUPPORT_H_
#pragma once

#ifndef nullptr
#define nullptr NULL
#endif
#include "../../BaseLib/math/conversion.h"
#include "../../BaseLib/motion/Liegroup.h"
#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix< double, 6, 6> CMatrix66;
typedef Eigen::Matrix< double, 6, 1> CVector6;
// Eigen::MatrixXd is a column-major type.
// So let's define a row-major type (RMatrixX). you can always copy a MatrixXd to a RMatrixXd
typedef Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > RMatrixXd;
typedef Eigen::Map<RMatrixXd  , Eigen::Unaligned, Eigen::Stride<Eigen::Dynamic, 1> > RMatrixXdView;
typedef Eigen::Map<Eigen::MatrixXd  , Eigen::Unaligned, Eigen::Stride<Eigen::Dynamic, 1> > MatrixXdView;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMatrix33;
typedef Eigen::Matrix< double, 3,1> CVector3;

/****************************************************************
 * for matrixn <-> MatrixXd conversion without copying elements *
 ****************************************************************/
// the folowing four functions share the original memory.
// returns a row-major Eigen matrix (no element copying)
inline RMatrixXdView eigenView(matrixn const& m) { return RMatrixXdView((double*)&m(0,0), m.rows(), m.cols(), Eigen::Stride<Eigen::Dynamic, 1>(m._getStride(), 1));}
inline RMatrix33& eigenView(matrix3 const& m) { ASSERT(sizeof(double)*9==sizeof(RMatrix33));  return *((RMatrix33*) &m);}
inline CVector3& eigenView(vector3 const& m) { ASSERT(sizeof(double)*3==sizeof(CVector3));  return *((CVector3*) &m);}
inline CVector6& eigenView(Liegroup::se3 const& m) { ASSERT(sizeof(double)*6==sizeof(CVector6));  return *((CVector6*) &m);}
inline matrix3& trlView(RMatrix33 const& m) { ASSERT(sizeof(double)*9==sizeof(RMatrix33));  return *((matrix3*) &m);}
inline vector3& trlView(CVector3 const& m) { ASSERT(sizeof(double)*3==sizeof(CVector3));  return *((vector3*) &m);}
inline Liegroup::se3& trl_se3(CVector6 const& m) { ASSERT(sizeof(double)*6==sizeof(CVector6));  return *((Liegroup::se3*) &m);}

inline vector3 toVector3(Eigen::VectorXd const& v, int i) { return vector3(v[i], v[i+1], v[i+2]);}


// returns a column-major Eigen matrix (no element copying but returns a transposed matrix.)
inline MatrixXdView eigenTView(matrixn const& m) { return MatrixXdView((double*)&m(0,0), m.cols(), m.rows(), Eigen::Stride<Eigen::Dynamic, 1>(m._getStride(), 1));}
inline MatrixXdView eigenTView(vectorn const& m) { return MatrixXdView((double*)&m(0), m.size(), 1, Eigen::Stride<Eigen::Dynamic, 1>(m._getStride(), 1));}

namespace Liegroup {
	// Ad == dAd'
	inline CMatrix66 Ad(transf const& b) { matrixn m(6,6); Liegroup::dAd(m, b); return eigenTView(m);}
	inline CMatrix66 dAd(transf const& b) { matrixn m(6,6); Liegroup::dAd(m, b); return eigenView(m);}
	inline CMatrix66 dot_Ad(transf const& b, matrix4 const& dotB) { matrixn m(6,6); Liegroup::dot_dAd(m, b, dotB); return eigenTView(m);}
	inline CMatrix66 dot_dAd(transf const& b, matrix4 const& dotB) { matrixn m(6,6); Liegroup::dot_dAd(m, b, dotB); return eigenView(m);}
}

// returns a matrixn
inline matrixnView matView(RMatrixXd const& x) { Msg::verify(x.rows()==1 || &x(1,0)-&x(0,0)==x.cols(), "Stride!=cols(). I don't know how to obtain the stride of Eigen::Matrix. So I just assumed this but..."); return matrixnView((double*)&x(0,0), x.rows(), x.cols(), x.cols());}
// returns a matrixn (but is transposed) 
inline matrixnView matTView(Eigen::MatrixXd const& x) { Msg::verify(x.cols()==1 || &x(0,1)-&x(0,0)==x.rows(), "Stride!=rows(). I don't know how to obtain the stride of Eigen::Matrix. So I just assumed this but..."); return matrixnView((double*)&x(0,0), x.cols(), x.rows(), x.rows());}

/***************************************
 * for vectorn <-> VectorXd conversion *
 ***************************************/
// if v1._getStride()==n (!=1), use Map<VectorXd, 0, Stride<Dynamic, n> > instead.
inline Eigen::Map<Eigen::VectorXd> eigenView(vectorn const& v1) { Msg::verify(v1._getStride()==1,"cannot be converted to Eigen::Map<VectorXd>"); return Eigen::VectorXd::Map((double*)&v1[0], v1.size());}
inline vectornView vecView(Eigen::VectorXd const& x) { return vectornView((double*)&x(0), x.size(), 1); }

// all the following functions copy the data. Use eigenView for referencing.
// convert to BaseLib type.
inline vector3 toBase(Eigen::Vector3d const & v) { return vector3(v(0), v(1), v(2));}
inline quater toBase(Eigen::Quaterniond const& q) { return quater(q.w(), q.x(), q.y(), q.z());}
//inline matrix3 toBase(Eigen::Matrix33d const & v) { matrix3 out; for(int i=0; i<3; i++) for(int j=0; j<3; j++) out.m[i][j]=m(i,j); return out; }

// convert to Eigen types.
inline Eigen::Vector3d toEigen ( vector3 const & v) { return Eigen::Vector3d(v.x, v.y, v.z);}
inline Eigen::Quaterniond toEigen ( quater const & v) { return Eigen::Quaterniond(v.w,v.x, v.y, v.z);}

// usage:
// matrixn m0(10,10);
// RMatrixXdView m=eigenView(m0);
// m(0,0)=7; // now you can use eigen functions on m.
// printf("%f", m0(0,0));

// matrixn a;
// a.setSize(10,10);
// 
// for(int i=0; i<10; i++)
// 	for(int j=0; j<10; j++)
// 		a(i,j)=i*10+j;
// std::cout<<a.output().ptr()<<std::endl;
// std::cout<<eigenView(a)<<std::endl;
// std::cout<<eigenView(a.range(3,8, 5,7))<<std::endl;
//
/*
int array[24];
for(int i = 0; i < 24; ++i) array[i] = i;
cout << Map<MatrixXi, 0, Stride<Dynamic,2> >
         (array, 3, 3, Stride<Dynamic,2>(8, 2))
     << endl;
Output:

 0  8 16
 2 10 18
 4 12 20
 */
/*
 A simple program that demonstrates the Eigen library.
 The program defines a random symmetric matrix
 and computes its eigendecomposition.
 For further details read the Eigen Reference Manual

#include <stdlib.h>
#include <time.h>
#include <string.h>

// the following two are needed for printing
#include <iostream>
#include <iomanip>
// The Eigen include files         
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
	int M = 3, N = 5;
	MatrixXd X(M,N); // Define an M x N general matrix

	// Fill X by random numbers between 0 and 9
	// Note that indexing into matrices in NewMat is 1-based!
	srand(time(NULL));
	for (int i = 0; i < M; ++i) {
		for (int j = 0; j < N; ++j) {
			X(i,j) = rand() % 10;
		}
	}

	MatrixXd C;
	C = X * X.transpose(); // fill in C by X * X^t. 

	cout << "The symmetrix matrix C" << endl;
    cout << C << endl;
	

	// compute eigendecomposition of C
    SelfAdjointEigenSolver<MatrixXd> es(C);

    MatrixXd D = es.eigenvalues().asDiagonal();
    MatrixXd V = es.eigenvectors();

	// Print the result
	cout << "The eigenvalues matrix:" << endl;
	cout << D << endl;
	cout << "The eigenvectors matrix:" << endl;
	cout << V << endl;

	// Check that the first eigenvector indeed has the eigenvector property
	VectorXd v1(3);
	v1(0) = V(0,0);
	v1(1) = V(1,0);
	v1(2) = V(2,0);

	VectorXd Cv1 = C * v1;
	VectorXd lambda1_v1 = D(0) * v1;

	cout << "The max-norm of the difference between C*v1 and lambda1*v1 is " << endl;
	cout << Cv1.cwiseMax(lambda1_v1) << endl << endl;

	// Build the inverse and check the result
	MatrixXd Ci = C.inverse();
	MatrixXd I  = Ci * C;

	cout << "The inverse of C is" << endl;
	cout << Ci << endl;
	cout << "And the inverse times C is identity" << endl;
	cout << I << endl;

	// Example for multiple solves 
	VectorXd r1(3), r2(3);
	for (int i = 0; i < 3; ++i) {
		r1(i) = rand() % 10;
		r2(i) = rand() % 10;
	}
    ColPivHouseholderQR<MatrixXd> qr(C); // decomposes C
	VectorXd s1 = qr.solve(r1);
	VectorXd s2 = qr.solve(r2);

	cout << "solution for right hand side r1" << endl;
	cout << s1 << endl;
	cout << "solution for right hand side r2" << endl;
	cout << s2 << endl;
	
	return 0;
}
*/
// deprecated. use eigenView instead.
inline RMatrixXdView matEigenView(matrixn const& m) { return RMatrixXdView((double*)&m(0,0), m.rows(), m.cols(), Eigen::Stride<Eigen::Dynamic, 1>(m._getStride(), 1));}
#endif
