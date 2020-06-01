#include "stdafx.h"
#include "mathclass.h"
#include "Operator.h"
#include "Operator_NR.h"
#include "Filter.h"
//#include "nr/nr.h"
#include "../PhysicsLib/TRL/eigenSupport.h"

namespace m
{
	void LUsolve(matrixn const & A, vectorn const& b, vectorn& x)
	{
		Msg::error("not implemented yet!");
/*		DP p;
		matrixn LU(A);
		Vec_INT indx(A.rows());
		NR::ludcmp(LU,indx,p);
		x=b;
		NR::lubksb(LU,indx,x);
		*/
	}

	void Diaginvert(vectorn& out, const vectorn& in, m_real& log_det)
	{
		out.each1(s1::LOG, in);
		log_det=out.sum();
		out.each1(s1::INVERSE, in);
	}

	void LUinvert(matrixn& F, const matrixn& E) 
	{
		/* transfer matrix for inversion into F */

		ASSERT(E.rows()==E.cols());
		F.setSameSize(E);


		eigenView(F)=eigenView(E).inverse();

		/*
		matrixn C_temp;
		vectorn lu_col;
		C_temp=E;
		lu_col.setSize(E.rows());
		Vec_INT indx(E.rows());
		m_real d;
		NR::ludcmp(C_temp, indx, d);
		int n=E.rows();

		for(int j=0;j<n;j++)
		{
			lu_col.setAllValue(0);
			lu_col[j]=1.0;
			NR::lubksb(C_temp,indx,lu_col);
			for(int i=0;i<n;i++) F[i][j] = lu_col[i];
		}
		*/
	}

	void LUinvert(matrixn& F, const matrixn& E, m_real& log_det)
	{
		/* transfer matrix for inversion into F */

		ASSERT(E.rows()==E.cols());
		F.setSameSize(E);

		matrixn C_temp;
		vectorn lu_col;
		C_temp=E;
		lu_col.setSize(E.rows());
		log_det = 0.0;

		Msg::error("luinvert");
		/*
		Vec_INT indx(E.rows());
		m_real d;
		NR::ludcmp(C_temp, indx, d);
		int n=E.rows();

		for(int i=0; i < n; ++i)
		{
			log_det += log(fabs(C_temp[i][i]));
			d       *= ( C_temp[i][i] > 0.0 ) ? 1.0 : - 1.0 ; 
		}
		if( d < 0.0 ) printf("WARNING - negative eigenvalues\n");

		for(int j=0;j<n;j++)
		{
			lu_col.setAllValue(0);
			lu_col[j]=1.0;
			NR::lubksb(C_temp,indx,lu_col);
			for(int i=0;i<n;i++) F[i][j] = lu_col[i];
		}
		*/
	}
	void LUinvert(matrixn& F, const matrixn& E, m_real & d_man, int& d_exp)
	{
		ASSERT(E.rows()==E.cols());
		F.setSameSize(E);

		matrixn C_temp;		
		vectorn col;
		C_temp=E;
		
		int n=E.rows();

		col.setSize(n);

		
		Msg::error("luinvert");
		/*
		Vec_INT indx(n);
		
		
		d_exp = 0;
		NR::ludcmp(C_temp,indx,d_man);


		for(int j=0; j<n; j++) 
		{
			d_man *= C_temp[j][j];
			while( fabs(d_man)>10 ) 
			{
			  d_man = d_man/10;
			  d_exp++;
			}
			while( (fabs(d_man)<0.1)&&(fabs(d_man)>0) ) {
			  d_man = d_man*10;
			  d_exp--;
			}
		}

		for(int j=0; j<n; j++) 
		{
			col.setAllValue(0);
			col[j]=1.0;
			NR::lubksb(C_temp, indx,col);
			for(int i=0; i<n; i++) F[i][j] = col[i];
		}
		*/
	}

	// covariance * numData: usually called the matrix S. 
	void covarianceN(vectorn& mean, matrixn& c, const matrixn& a) 
	{
		mean.aggregateEachColumn(CAggregate::AVG, a);

		matrixn zeroMeaned;
		zeroMeaned.setSize(a.rows(), a.cols());
		for (int i=0;i<a.rows();i++)
			zeroMeaned.row(i).sub(a.row(i), mean);

		//zeroMeaned.each2(&vectorn::sub, a, mean);
	//	m::each2(	zeroMeaned, &vectorn::sub, a, mean);

		int dimension=a.cols();
		m_real numData=a.rows();

		c.setSize(dimension, dimension);

		for(int i=0; i<dimension; i++)
		{
			for(int j=0; j<dimension; j++)
			{
				// 곱한거의 average
				c[i][j]=(zeroMeaned.column(i)%zeroMeaned.column(j));///(numData);
			}
		}
	}

	void pseudoInverse(matrixn& ret, const matrixn& a)
	{
		if(a.rows() > a.cols())
		{
			static matrixn inv, ata, at;
			at.transpose(a);
			ata.mult(at,a);
			m::LUinvert(inv,ata);
			ret.mult(inv, at);//inv(A'A)A'
		}
		else
		{
			static matrixn at, inv, aat;
			at.transpose(a);
			aat.mult(a,at);
			m::LUinvert(inv,aat);
			ret.mult(at, inv);//A'inv(AA')
		}
	}
	void PIsolve(matrixn const & A, vectorn const& b, vectorn& x)
	{
		matrixn tmp;
		m::pseudoInverse(tmp, A);

		//Ax=b
		//x=pinvA*b

		x.multmat(tmp,b);
	}
	m_real determinant(const matrixn& E)
	{
		/* transfer matrix for inversion into F */
		ASSERT(E.rows()==E.cols());

		if(E.rows()==1)
			return E[0][0];
		static matrixn C_temp;
		static vectorn lu_col;
		C_temp=E;
		lu_col.setSize(E.rows());
		m_real det=1.0;

		Msg::error("LUinvert not implemented yet!");
		/*
		Vec_INT indx(E.rows());
		m_real d=DBL_MAX;
		NR::ludcmp(C_temp, indx, d);
		int n=E.rows();

		if(d==DBL_MAX)	// singular matrix
			return 0.0;

		for(int i=0; i < n; ++i)
		{
			det*= C_temp[i][i];
		}
		return det*d;
		*/
		return 0;
	}
	void eigenDecomposition(matrixn const& cov, vectorn & d, matrixn & v, int method)
	{
		Msg::error("eigsrt");
		/*
		const bool sort=true;

		switch(method)
		{
			case 0:
				{
					int nrot;
					matrixn a=cov;
					NR::jacobi(a, d,v, nrot);
				}
				break;
			case 1:
				{
					v=cov;
					vectorn e;
					d.setSize(v.rows());
					e.setSize(v.rows());
					NR::tred2(v, d, e);
					NR::tqli(d, e, v);
				}
				break;
		}
		if(sort)
			NR::eigsrt(d,v);
			*/
	}
	void cofactor(matrixn& c, const matrixn& a) 
	{
		c.setSameSize(a);

		matrixn det(a.rows()-1, a.rows()-1);
		for(int i=0; i<a.rows(); i++)
		{
			for(int j=0; j<a.cols(); j++)
			{
				for(int k=0; k<i; k++)
				{
					for(int l=0; l<j; l++)
						det[k][l]=a[k][l];
					for(int l=j+1; l<a.cols(); l++)
						det[k][l-1]=a[k][l];
				}
				for(int k=i+1; k<a.rows(); k++)
				{
					for(int l=0; l<j; l++)
						det[k-1][l]=a[k][l];
					for(int l=j+1; l<a.cols(); l++)
						det[k-1][l-1]=a[k][l];
				}

				c[i][j]=m::determinant(det);
				if((i+j)%2==1)
					c[i][j]*=-1.0;
			}
		}
	}
	void eigenVectors(matrixn& EVectors, const matrixn& mat, vectorn& eigenValues)
	{
		Msg::error("eigvec");
		/*
		matrixn symData(mat);
		vectorn e;
		eigenValues.setSize(mat.rows());
		e.setSize(mat.rows());
		NR::tred2(symData, eigenValues, e);
		NR::tqli(eigenValues, e,symData);

		int dim=eigenValues.size();

		EVectors.setSize(dim,dim);

		// Now each row contains EigenVector
		for (int i = 0; i < dim; i++) {
			for (int j = 0; j < dim; j++) {
				EVectors[i][j] = symData[j][i];
			}
		}
		*/
	}
}
void m::SVdecompose(matrixn& in_u, vectorn & s, matrixn &v)
{
	Msg::error("svdcmp");
	/*
	// in=u*diag(s)*v
	int m = in_u.rows();
	int n = in_u.cols();
	s.setSize(n);
	v.setSize(n,n);
	NR::svdcmp(in_u, s, v);
	*/
}

void m::SVinverse( matrixn& in_u, matrixn &mat )
{
	Msg::error("svdinv");
	/*
	int m = in_u.rows();
	int n = in_u.cols();

	static matrixn V; V.setSize( n, n );
	static vectorn w; w.setSize( n );

	static vectorn b; b.setSize( m );
	static vectorn x; x.setSize( n );

	mat.setSize( n, m );

	NR::svdcmp( in_u, w, V );
	for( int j=0; j<m; j++ )
	{
		for( int i=0; i<m; i++ ) b[i] = 0;
		b[j] = 1.0;

		NR::svbksb(in_u, w, V, b, x );

		for( int i=0; i<n; i++ )
			mat[i][j] = x[i];
	}
	*/
}

