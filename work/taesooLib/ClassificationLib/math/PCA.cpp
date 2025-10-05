#include "stdafx.h"
#include "PCA.H"
#include "../BaseLib/math/Operator.h"
#define EigenValue_Threshold 30.0
//#include "Global2.h"

//extern void MESSAGE(char *format, ...);

PCA::PCA()
{
	forceReducedDim=false;
	m_ReducedDim = 0;
	m_noSamples = 0;
	m_ReducedData = NULL;
	m_bPercentThreshold = true;
	m_error_goal = 0.95;
	m_EigenValues = NULL;
}

void PCA::setReducedDim(int dim)
{
	forceReducedDim=true;
	m_ReducedDim=dim;
}
PCA::PCA(double error_goal)
{
	forceReducedDim=false;
	m_ReducedDim = 0;
	m_noSamples = 0;
	m_ReducedData = NULL;
	m_bPercentThreshold = true;
	m_error_goal = error_goal;
	m_EigenValues = NULL;
}

PCA::~PCA()
{
	if (m_ReducedData != NULL) 
		for (int i = 0; i < m_noSamples; i++)	delete m_ReducedData[i];
	delete [] m_ReducedData;

	if(m_EigenValues != NULL) delete [] m_EigenValues;
}

void PCA::getPCA(matrixn& OrgTemplateData, bool bOutput)
{
	int i, j, k;

	int dim=OrgTemplateData.cols();
	m_noSamples = OrgTemplateData.rows();
	double **InputData = new double*[dim];
	double **SymData = new double*[dim];
	double **EVectors = new double*[dim];
	double *EValues = new double[dim];
	double *e = new double[dim];
	m_MeanData.setSize(dim);

	for (i = 0; i < dim; i++) {
		InputData[i] = new double[m_noSamples];
		SymData[i] = new double[dim];
		EVectors[i] = new double[dim];
		for (j = 0; j < m_noSamples; j++) InputData[i][j] = OrgTemplateData[j][i]; 
	}

	// Get Covariance Matrix
	GetCovMatrix(InputData, dim, m_noSamples, SymData, m_MeanData.dataPtr());

	// Get EigenValues && EigenVectors from the Covariance Matrix
	tred2(SymData, EValues, e, dim);
	tqli(EValues, e, SymData, dim);

	// Now each row contains EigenVector
	for (i = 0; i < dim; i++) {
		for (j = 0; j < dim; j++) {
			EVectors[i][j] = SymData[j][i];
		}
	}

	// Insertion Sort for EigenValues & EigenVectors, sort eigenvalues from highest to lowest
	double *EVtemp = new double[dim];
	for (i = 1; i < dim; i++) {
		double t = EValues[i];
		for (k = 0; k < dim; k++) EVtemp[k] = EVectors[i][k];
		int j;
		for (j = i - 1; j >= 0 && t > EValues[j]; j--) {
			EValues[j + 1] = EValues[j];
			for (k = 0; k < dim; k++) EVectors[j + 1][k] = EVectors[j][k];
		}
		EValues[j + 1] = t;
		for (k = 0; k < dim; k++) EVectors[j + 1][k] = EVtemp[k];
	}

	
	m_OriginalDim = dim;
	if(m_EigenValues != NULL) delete m_EigenValues;	m_EigenValues = new double [dim];
	for(i=0; i<dim; i++) m_EigenValues[i] = EValues[i];

	if(!m_bPercentThreshold)
	{
		if(m_ReducedDim <0)
			m_ReducedDim*=-1;
		else
		{
			m_ReducedDim = dim;
			for (i = 0; i < dim; i++) {
				if (EValues[i] < EigenValue_Threshold)	{
					m_ReducedDim = i;
					break;
				}
			}

		}
	}
	else
	{
		if(!forceReducedDim)
		{
		m_ReducedDim = dim;
		double sum1 = 0.0f;
		double sum2 = 0.0f;
		for(i=0; i<dim; i++)
			sum1 += EValues[i];
		for(i=dim-1; i>0; i--)
		{
			sum2 += EValues[i];
			if(sum2/sum1 > (1.0f-m_error_goal))
			{
				m_ReducedDim = i+1;
				break;
			}
		}
		}
	}
	double sum1 = 0.0f;
	double sum2 = 0.0f;
	for(i=0; i<dim; i++)
		sum1 += EValues[i];
	for(i=0; i<m_ReducedDim; i++)
		sum2 += EValues[i];
	m_construction_error = sum2/sum1;


	if(bOutput)
	{
		Msg::print("Parameter Dimensions: %d ---> Reduced Dimensions: %d\r\n", dim, m_ReducedDim);
		for (i = 0; i < dim; i++)
		{
			Msg::print("%f\r\n", EValues[i]);
		}
	}

	double **new_EVectors = new double*[m_ReducedDim];
	for (i = 0; i < m_ReducedDim; i++) new_EVectors[i] = new double[dim];

	for (i = 0; i < m_ReducedDim; i++) {
		for (j = 0; j < dim; j++) {
			new_EVectors[i][j] = EVectors[i][j];
		}
	}

	m_FeatureVector.setSize(m_ReducedDim, dim);
	matrixn MeanAdjustedData(dim, m_noSamples);
	matrixn OutputData(m_ReducedDim, m_noSamples);

	for (i = 0; i < m_ReducedDim; i++)
		for (j = 0; j < dim; j++)
			m_FeatureVector[i][j] = (double)new_EVectors[i][j];

	for (i = 0; i < dim; i++)
		for (j = 0; j < m_noSamples; j++)
			MeanAdjustedData[i][j] = (double)InputData[i][j];

	OutputData.mult(m_FeatureVector, MeanAdjustedData);
	
	m_ReducedData = new double*[m_noSamples];
	for (i = 0; i < m_noSamples; i++) m_ReducedData[i] = new double[m_ReducedDim];

	for (i = 0; i < m_ReducedDim; i++) {
		for (j = 0; j < m_noSamples; j++) {
			m_ReducedData[j][i] = OutputData[i][j];
		}
	}

	for (i = 0; i < m_ReducedDim; i++) {
		delete [] new_EVectors[i];
	}
	delete [] new_EVectors;

	for (i = 0; i < dim; i++) {
		delete [] InputData[i];
		delete [] SymData[i];
		delete [] EVectors[i];
	}
	delete [] EVtemp;
	delete [] EVectors;
	delete [] EValues;
	delete [] e;
	delete [] InputData;
	delete [] SymData;
}

void PCA::EigenData(matrixn& data)
{
	// eigenVector*MaxEigenValue+meanData
	data.mult(m_FeatureVector, sqrt(m_EigenValues[0]));
	for(int i=0; i<data.rows(); i++)
		data.row(i).lval()+= m_MeanData;
}

void PCA::GetReducedData(matrixn& reducedData)
{
	reducedData.setSize(m_noSamples, m_ReducedDim);
	for(int i=0; i<m_noSamples; i++)
	{
		for(int j=0; j<m_ReducedDim; j++)
		{
			reducedData[i][j]=m_ReducedData[i][j];
		}
	}
}

void PCA::GetReducedData(int i, vectorn& reducedVec)
{
	reducedVec.setSize(m_ReducedDim);
	for(int j=0; j<m_ReducedDim; j++)
	{
		reducedVec[j]=m_ReducedData[i][j];
	}
}

void PCA::Projection(const vectorn& origVec, vectorn& reducedVec) const
{
	vectorn zeroMeaned;
	zeroMeaned.sub(origVec, m_MeanData);
	reducedVec.multmat(m_FeatureVector, zeroMeaned);
}

void PCA::Projection(const matrixn& origVecs, matrixn& reducedVecs) const
{
	reducedVecs.setSize(origVecs.rows(), m_ReducedDim);
	for(int i=0; i<origVecs.rows(); i++)
	{
		reducedVecs.row(i).multmat(m_FeatureVector, origVecs.row(i));
	}
}
// Create n * n covariance matrix from given n * m data matrix. 
//void GetCovMatrix(matrixn InputData, int n, int m, matrixn SymData, matrixn MeanData)
void PCA::GetCovMatrix(double **InputData, int n, int m, double **SymData, double *mean)
{
	int i, j, j1, j2;

	// Determine mean of column vectors of input data matrix 
	for (i = 0; i < n; i++) {
		mean[i] = 0.0;
		for (j = 0; j < m; j++) {
			mean[i] += (double)InputData[i][j];
        }
		mean[i] /= (double)m;
    }

	// Center the column vectors. 
	for (i = 0; i < n; i++) {
		for (j = 0; j < m; j++) {
			InputData[i][j] -= mean[i];
        }
    }

	// Calculate the n * n covariance matrix. 
	for (j1 = 0; j1 < n; j1++) {
		for (j2 = 0; j2 < n; j2++) {
			SymData[j1][j2] = 0.0;
			for (i = 0; i < m; i++) {
				SymData[j1][j2] += InputData[j1][i] * InputData[j2][i];
            }
			SymData[j1][j2] /= (m - 1);
        }
    }
}

void PCA::tred2(double **a, double *d, double *e, int size)
{
	int l,k,j,i;
	double scale,hh,h,g,f;

	int n=size;
	for (i=n-1;i>0;i--) {
		l=i-1;
		h=scale=0.0;
		if (l > 0) {
			for (k=0;k<l+1;k++)
				scale += fabs(a[i][k]);
			if (scale == 0.0)
				e[i]=a[i][l];
			else {
				for (k=0;k<l+1;k++) {
					a[i][k] /= scale;
					h += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i]=scale*g;
				h -= f*g;
				a[i][l]=f-g;
				f=0.0;
				for (j=0;j<l+1;j++) {
				// Next statement can be omitted if eigenvectors not wanted
					a[j][i]=a[i][j]/h;
					g=0.0;
					for (k=0;k<j+1;k++)
						g += a[j][k]*a[i][k];
					for (k=j+1;k<l+1;k++)
						g += a[k][j]*a[i][k];
					e[j]=g/h;
					f += e[j]*a[i][j];
				}
				hh=f/(h+h);
				for (j=0;j<l+1;j++) {
					f=a[i][j];
					e[j]=g=e[j]-hh*f;
					for (k=0;k<j+1;k++)
						a[j][k] -= (f*e[k]+g*a[i][k]);
				}
			}
		} else
			e[i]=a[i][l];
		d[i]=h;
	}
	// Next statement can be omitted if eigenvectors not wanted
	d[0]=0.0;
	e[0]=0.0;
	// Contents of this loop can be omitted if eigenvectors not
	//	wanted except for statement d[i]=a[i][i];
	for (i=0;i<n;i++) {
		l=i;
		if (d[i] != 0.0) {
			for (j=0;j<l;j++) {
				g=0.0;
				for (k=0;k<l;k++)
					g += a[i][k]*a[k][j];
				for (k=0;k<l;k++)
					a[k][j] -= g*a[k][i];
			}
		}
		d[i]=a[i][i];
		a[i][i]=1.0;
		for (j=0;j<l;j++) a[j][i]=a[i][j]=0.0;
	}
}

void PCA::tqli(double *d, double *e, double **z, int size)
{
	int m,l,iter,i,k;
	double s,r,p,g,f,dd,c,b;

	int n=size;
	for (i=1;i<n;i++) e[i-1]=e[i];
	e[n-1]=0.0;
	for (l=0;l<n;l++) {
		iter=0;
		do {
			for (m=l;m<n-1;m++) {
				dd=fabs(d[m])+fabs(d[m+1]);
				if (fabs(e[m])+dd == dd) break;
			}
			if (m != l) {
				if (iter++ == 30) {
//					MESSAGE("Too many iterations in tqli");
					return;
				}
				g=(d[l+1]-d[l])/(2.0*e[l]);
				r=pythag(g,1.0);
				g=d[m]-d[l]+e[l]/(g+SIGN(r,g));
				s=c=1.0;
				p=0.0;
				for (i=m-1;i>=l;i--) {
					f=s*e[i];
					b=c*e[i];
					e[i+1]=(r=pythag(f,g));
					if (r == 0.0) {
						d[i+1] -= p;
						e[m]=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d[i+1]-p;
					r=(d[i]-g)*s+2.0*c*b;
					d[i+1]=g+(p=s*r);
					g=c*r-b;
					// Next loop can be omitted if eigenvectors not wanted
					for (k=0;k<n;k++) {
						f=z[k][i+1];
						z[k][i+1]=s*z[k][i]+c*f;
						z[k][i]=c*z[k][i]-s*f;
					}
				}
				if (r == 0.0 && i >= l) continue;
				d[l] -= p;
				e[l]=g;
				e[m]=0.0;
			}
		} while (m != l);
	}
}

double PCA::pythag(const double a, const double b)
{
	double absa,absb;

	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+SQR(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

