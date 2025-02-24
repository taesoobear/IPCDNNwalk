#include <vector>
#include <math.h>
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"

using namespace std;

void put_se3_to_matrix(RMatrix &M, const se3 &S, int idx)
{ 
	M[idx++] = S[0]; M[idx++] = S[1]; M[idx++] = S[2]; 
	M[idx++] = S[3]; M[idx++] = S[4]; M[idx++] = S[5]; 
}

void put_se3_to_matrix(RMatrix &M, const se3 &S, int r, int c) 
{ 
	put_se3_to_matrix(M, S, r+c*M.RowSize()); 
}

void put_dse3_to_matrix(RMatrix &M, const dse3 &S, int idx)
{ 
	M[idx++] = S[0]; M[idx++] = S[1]; M[idx++] = S[2]; 
	M[idx++] = S[3]; M[idx++] = S[4]; M[idx++] = S[5]; 
}

void put_dse3_to_matrix(RMatrix &M, const dse3 &S, int r, int c) 
{ 
	put_dse3_to_matrix(M, S, r+c*M.RowSize()); 
}

void put_Vec3_to_matrix(RMatrix &M, const Vec3 &v, int idx)
{
	M[idx++] = v[0]; M[idx++] = v[1]; M[idx++] = v[2]; 
}

void put_Vec3_to_matrix(RMatrix &M, const Vec3 &v, int r, int c)
{
	put_Vec3_to_matrix(M, v, r+c*M.RowSize());
}

RMatrix Ad(const SE3 &T)
{
	RMatrix m, R, skew_p;

	R = convert_to_RMatrix(T.GetRotation()); 
	skew_p = matrix_skew(T.GetPosition());

	m = Zeros(6,6);

	m.Push(0, 0, R);
	m.Push(3, 0, skew_p*R);
	m.Push(3, 3, R);

	return m;
}

RMatrix dAd(const SE3 &T)
{
	return ~Ad(T);
}

RMatrix matrix_a_at(const Vec3 &a)
{
	RMatrix m(3,3);

	m(0,0) = a[0]*a[0];		m(0,1) = a[0]*a[1];		m(0,2) = a[0]*a[2];
	m(1,0) = a[0]*a[1];		m(1,1) = a[1]*a[1];		m(1,2) = a[1]*a[2];
	m(2,0) = a[0]*a[2];		m(2,1) = a[2]*a[1];		m(2,2) = a[2]*a[2];

	return m;
}

RMatrix matrix_a_bt(const Vec3 &a, const Vec3 &b)
{
	int i, j;
	RMatrix m(3,3);

	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			m(i,j) = a[i]*b[j];
		}
	}

	return m;
}

void matrix_A_At(RMatrix &re, const RMatrix &A)
{
	int i, j, k, n;

	n = A.RowSize();
	re.ReNew(n,n);

	for (i=0; i<n; i++) {
		for (j=0; j<=i; j++) {
			re(i,j) = 0.0;
			for (k=0; k<A.ColSize(); k++) {
				re(i,j) += A(i,k)*A(j,k);
			}
		}
	}

	for (i=0; i<n; i++) {
		for (j=i+1; j<n; j++) {
			re(i,j) = re(j,i);
		}
	}
}

void matrix_At_A(RMatrix &re, const RMatrix &A)
{
	int i, j, k, n;

	n = A.ColSize();
	re.ReNew(n,n);

	for (i=0; i<n; i++) {
		for (j=0; j<=i; j++) {
			re(i,j) = 0.0;
			for (k=0; k<A.RowSize(); k++) {
				re(i,j) += A(k,i)*A(k,j);
			}
		}
	}

	for (i=0; i<n; i++) {
		for (j=i+1; j<n; j++) {
			re(i,j) = re(j,i);
		}
	}
}

void matrix_At_W_A(RMatrix &re, const RMatrix &A, const RMatrix &w)
{
	int i, j, k, n;

	n = A.ColSize();
	re.ReNew(n,n);

	for (i=0; i<n; i++) {
		for (j=0; j<=i; j++) {
			re(i,j) = 0.0;
			for (k=0; k<A.RowSize(); k++) {
				re(i,j) += A(k,i)*A(k,j)*w[k];
			}
		}
	}

	for (i=0; i<n; i++) {
		for (j=i+1; j<n; j++) {
			re(i,j) = re(j,i);
		}
	}
}

void matrix_AtB_plus_BtA(RMatrix &re, const RMatrix &A, const RMatrix &B)
{
	int i, j, k, n;

	// A and B should have the same size
	if ( A.RowSize() != B.RowSize() || A.ColSize() != B.ColSize() ) return;

	n = A.ColSize();
	re.ReNew(n,n);

	for (i=0; i<n; i++) {
		for (j=0; j<=i; j++) {
			re(i,j) = 0.0;
			for (k=0; k<A.RowSize(); k++) {
				re(i,j) += A(k,i)*B(k,j) + B(k,i)*A(k,j);
			}
		}
	}

	for (i=0; i<n; i++) {
		for (j=i+1; j<n; j++) {
			re(i,j) = re(j,i);
		}
	}
}

bool remove_zero_row_AxEqualB(RMatrix &A, RMatrix &b)
{
	if ( A.RowSize() != b.RowSize() || b.ColSize() != 1 ) return false;

	int i, j;
	double row_sum;
	vector<int> idx_nonzero_row;
	for (i=0; i<A.RowSize(); i++)
	{
		row_sum = 0.0;
		for (j=0; j<A.ColSize(); j++) row_sum += A(i,j)*A(i,j);
		if ( row_sum > 1E-10 ) 
		{
			idx_nonzero_row.push_back(i);
		}
		else
		{
			if ( b(i,0) > 1E-10 ) return false;
		}
	}

	if ( idx_nonzero_row.size() == A.RowSize() ) return true;

	RMatrix A_tmp(int(idx_nonzero_row.size()), A.ColSize()), b_tmp(int(idx_nonzero_row.size()), 1);
	for (i=0; i<int(idx_nonzero_row.size()); i++)
	{
		A_tmp.Push(i, 0, A.Sub(idx_nonzero_row[i], idx_nonzero_row[i], 0, A.ColSize()-1));
		b_tmp.Push(i, 0, b.Sub(idx_nonzero_row[i], idx_nonzero_row[i], 0, 0));
	}
	A = A_tmp;
	b = b_tmp;

	return true;
}

