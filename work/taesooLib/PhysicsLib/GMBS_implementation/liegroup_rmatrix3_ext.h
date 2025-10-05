//================================================================================
//         Addon function for liegroup.h and rmatrix3j.h
//                                                                     release 0.1 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _ADDON_FUNCTIONS_FOR_LIEGROUP_AND_RMATRIX3_
#define _ADDON_FUNCTIONS_FOR_LIEGROUP_AND_RMATRIX3_

#include <math.h>
#include "rmatrix3j.h"
#include "liegroup.h"


void put_se3_to_matrix(RMatrix &M, const se3 &S, int idx);
void put_se3_to_matrix(RMatrix &M, const se3 &S, int r, int c);

void put_dse3_to_matrix(RMatrix &M, const dse3 &S, int idx);
void put_dse3_to_matrix(RMatrix &M, const dse3 &S, int r, int c);

void put_Vec3_to_matrix(RMatrix &M, const Vec3 &v, int idx);
void put_Vec3_to_matrix(RMatrix &M, const Vec3 &v, int r, int c);

RMatrix Ad(const SE3 &T);									// return 6 by 6 matrix representation of adjoint mapping of T
RMatrix dAd(const SE3 &T);									// return 6 by 6 matrix representation of dual adjoint mapping of T

void Ad(RMatrix &re, const SE3 &T, const RMatrix &S);		// re = [Ad(T,s_0), ..., Ad(T,s_(num-1))] where s_i = (S[6*i+0],...,S[6*i+5])
void minus_Ad(RMatrix &re, const SE3 &T, const RMatrix &S);	// re = -[dAd(T,s_0), ..., dAd(T,s_(num-1))] where s_i = (S[6*i+0],...,S[6*i+5])
void dAd(RMatrix &re, const SE3 &T, const RMatrix &S);		// re = [dAd(T,s_0), ..., dAd(T,s_(num-1))] where s_i = (S[6*i+0],...,S[6*i+5])
void minus_dAd(RMatrix &re, const SE3 &T, const RMatrix &S);// re = -[Ad(T,s_0), ..., Ad(T,s_(num-1))] where s_i = (S[6*i+0],...,S[6*i+5])

void multAB(double *re, const double *a, const double *b, int a_row, int a_col, int b_row, int b_col); 
															// re = A*B where A = [a] = (a_row x a_col) and B = [b] = (b_row x b_col).
															// requirements: size(re) = a_row * b_col and a_col = b_row

//void multASAt(double *re, const double *a, const double *s, int a_row, int a_col, int s_row);
//															// re = A*S*At where A = [a] = (a_row x a_col), At = transpose of A, 
//															// and S = [s] = (s_row x s_row) = a symmetric matrix
//															// requirements: size(re) = a_row * a_row and a_col = s_row
// Not efficient at all!

RMatrix Ad(const SE3 &T, const RMatrix &J);					// return [Ad(T,J_1), ..., Ad(T,J_n)] where J = [J_1, ..., J_n] = 6 by n matrix

RMatrix dAd(const SE3 &T, const RMatrix &J);				// return [dAd(T,J_1), ..., dAd(T,J_n)] where J = [J_1, ..., J_n] = 6 by n matrix

RMatrix ad(const se3 &S, const RMatrix &J);					// return [ad(S,J_1), ..., ad(S,J_n)] where J = [J_1, ..., J_n] = 6 by n matrix

RMatrix ad(const RMatrix &S1, const RMatrix &S2);			// return ad(convert_to_se3(S1), S2)

RMatrix dad(const se3 &S, const RMatrix &J);				// return [dad(S,J_1), ..., dad(S,J_n)] where J = [J_1, ..., J_n] = 6 by n matrix

RMatrix Cross(const Vec3 &a, const RMatrix &B);				// return [Cross(a,b_1), ..., Cross(a,b_n)] where B = [b_1, ..., b_n] = 3 by n matrix

RMatrix convert_to_RMatrix(const Vec3 &p);

RMatrix convert_to_RMatrix(const SO3 &R);

RMatrix convert_to_RMatrix(const SE3 &T);

RMatrix convert_to_RMatrix(const se3 &s);

RMatrix convert_to_RMatrix(const dse3 &s);

RMatrix convert_to_RMatrix(const Inertia &I);

RMatrix convert_to_RMatrix(const AInertia &aI);

se3 convert_to_se3(const RMatrix &s);

dse3 convert_to_dse3(const RMatrix &m);

Vec3 convert_to_Vec3(const RMatrix &v);

RMatrix matrix_skew(const Vec3 &p);

RMatrix matrix_skew_skew(const Vec3 &p);									// return matrix_skew(p) * matrix_skew(p)

RMatrix matrix_a_at(const Vec3 &a);											// return a*~a which is a 3 x 3 matrix

RMatrix matrix_a_bt(const Vec3 &a, const Vec3 &b);							// return a*~b which is a 3 x 3 matrix

void matrix_At_A(RMatrix &re, const RMatrix &A);							// re = ~A*A

void matrix_A_At(RMatrix &re, const RMatrix &A);							// re = A*~A // not confirmed yet!

void matrix_At_W_A(RMatrix &re, const RMatrix &A, const RMatrix &w);		// re = ~A*diag(w)*A where w = A.RowSize() dimensional vector

void matrix_AtB_plus_BtA(RMatrix &re, const RMatrix &A, const RMatrix &B);	// re = ~A*B+~B*A

bool remove_zero_row_AxEqualB(RMatrix &A, RMatrix &b);						// remove zero rows in the linear equation, A*x=b


#include "liegroup_rmatrix3_ext.inl"

#endif

