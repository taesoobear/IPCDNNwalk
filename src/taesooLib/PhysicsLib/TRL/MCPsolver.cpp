
#include "physicsLib.h"
#include "Body.h"
#include "ContactForceSolver.h"
#include <limits>

using namespace std;
using namespace TRL;
#if 0
// precise setting
static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 500;
static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 10;
static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;
static const double DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR = 1.0e-3;
#else
//
// fast setting
static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 20;
static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 2;
static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;
static const double DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR = 1.0e-3;
#endif
#include "../../BaseLib/utility/QPerformanceTimer.h"
MCPsolver::MCPsolver()
{
	maxNumGaussSeidelIteration = DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION;
	numGaussSeidelInitialIteration = DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION;
	gaussSeidelMaxRelError = DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR;
}
void MCPsolver::initWorkspace()
{
	//mod
	if(globalNumContactNormalVectors!=0){
		_mem_contactIndexToMu.setSize(globalNumContactNormalVectors);
		_mem_mcpHi.setSize(globalNumContactNormalVectors);
		_CI2Mu=&_mem_contactIndexToMu[0];
		_mcpHi=&_mem_mcpHi[0];
	}
	//_mem_contactIndexToMu.setSize(globalNumContactNormalVectors);
	//_mem_mcpHi.setSize(globalNumContactNormalVectors);
	//_CI2Mu=&_mem_contactIndexToMu[0];
	//_mcpHi=&_mem_mcpHi[0];
}
void MCPsolver::solveMCPByProjectedGaussSeidel(const rmdmatrix& __M, const dvector& __b, dvector& __x)
{
	_M=&__M(0,0);
	_stride=&__M(1,0)-&__M(0,0);
	_b=&__b(0);
	_x=&__x(0);

	static const int loopBlockSize = DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK;

	if(numGaussSeidelInitialIteration > 0){
		solveMCPByProjectedGaussSeidelInitial(numGaussSeidelInitialIteration);
	}

	int numBlockLoops = maxNumGaussSeidelIteration / loopBlockSize;
	if(numBlockLoops==0) numBlockLoops = 1;


	double error = 0.0;
	int i=0;
	while(i < numBlockLoops){
		i++;
		solveMCPByProjectedGaussSeidelMain( loopBlockSize - 1);

		error = solveMCPByProjectedGaussSeidelErrorCheck();
		if(error < gaussSeidelMaxRelError){
			//cout << "stopped at " << i * loopBlockSize << endl;
			break;
		}
	}

}

inline void MCPsolver::solveMCPByProjectedGaussSeidelInitial
( const int numIteration)
{
	const int size = globalNumConstraintVectors + globalNumFrictionVectors;

	const double rstep = 1.0 / (numIteration * size);
	double r = 0.0;

	for(int i=0; i < numIteration; ++i){

		for(int j=0; j < globalNumContactNormalVectors; ++j){

			double sum = -M(j, j) * x(j);
			for(int k=0; k < size; ++k){
				sum += M(j, k) * x(k);
			}
			const double xx = (-b(j) - sum) / M(j, j);
			if(xx < 0.0){
				x(j) = 0.0;
			} else {
				x(j) = r * xx;
			}
			r += rstep;
			mcpHi(j) = contactIndexToMu(j) * x(j);
		}

		for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

			double sum = -M(j, j) * x(j);
			for(int k=0; k < size; ++k){
				sum += M(j, k) * x(k);
			}
			x(j) = r * (-b(j) - sum) / M(j, j);
			r += rstep;
		}

		{

			int contactIndex = 0;
			for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

				double sum = -M(j, j) * x(j);
				for(int k=0; k < size; ++k){
					sum += M(j, k) * x(k);
				}
				const double fx0 = (-b(j) - sum) / M(j, j);
				double& fx = x(j);

				++j;

				sum = -M(j, j) * x(j);
				for(int k=0; k < size; ++k){
					sum += M(j, k) * x(k);
				}
				const double fy0 = (-b(j) - sum) / M(j, j);
				double& fy = x(j);

				const double fmax = mcpHi(contactIndex);
				const double fmax2 = fmax * fmax;
				const double fmag2 = fx0 * fx0 + fy0 * fy0;

				if(fmag2 > fmax2){
					const double s = r * fmax / sqrt(fmag2);
					fx = s * fx0;
					fy = s * fy0;
				} else {
					fx = r * fx0;
					fy = r * fy0;
				}
				r += (rstep + rstep);
			}

		} 
	}
}
inline void MCPsolver::solveMCPByProjectedGaussSeidelMain
( const int numIteration)
{
	const int size = globalNumConstraintVectors + globalNumFrictionVectors;

	for(int i=0; i < numIteration; ++i){

		for(int j=0; j < globalNumContactNormalVectors; ++j){

			double sum = -M(j, j) * x(j);
			for(int k=0; k < size; ++k){
				sum += M(j, k) * x(k);
			}
			const double xx = (-b(j) - sum) / M(j, j);
			if(xx < 0.0){
				x(j) = 0.0;
			} else {
				x(j) = xx;
			}
			mcpHi(j) = contactIndexToMu(j) * x(j);
		}

		for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

			double sum = -M(j, j) * x(j);
			for(int k=0; k < size; ++k){
				sum += M(j, k) * x(k);
			}
			x(j) = (-b(j) - sum) / M(j, j);
		}


		{

			int contactIndex = 0;
			for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

				double sum = -M(j, j) * x(j);
				for(int k=0; k < size; ++k){
					sum += M(j, k) * x(k);
				}
				const double fx0 = (-b(j) - sum) / M(j, j);
				double& fx = x(j);

				++j;

				sum = -M(j, j) * x(j);
				for(int k=0; k < size; ++k){
					sum += M(j, k) * x(k);
				}
				const double fy0 = (-b(j) - sum) / M(j, j);
				double& fy = x(j);

				const double fmax = mcpHi(contactIndex);
				const double fmax2 = fmax * fmax;
				const double fmag2 = fx0 * fx0 + fy0 * fy0;

				if(fmag2 > fmax2){
					const double s = fmax / sqrt(fmag2);
					fx = s * fx0;
					fy = s * fy0;
				} else {
					fx = fx0;
					fy = fy0;
				}
			}

		} 
	}
}

inline double MCPsolver::solveMCPByProjectedGaussSeidelErrorCheck
()
{
	const int size = globalNumConstraintVectors + globalNumFrictionVectors;

	double error = 0.0;

	for(int j=0; j < globalNumConstraintVectors; ++j){

		double sum = -M(j, j) * x(j);
		for(int k=0; k < size; ++k){
			sum += M(j, k) * x(k);
		}
		double xx = (-b(j) - sum) / M(j, j);

		if(j < globalNumContactNormalVectors){
			if(xx < 0.0){
				xx = 0.0;
			}
			mcpHi(j) = contactIndexToMu(j) * xx;
		}
		double d = fabs(xx - x(j));
		if(xx > numeric_limits<double>::epsilon()){
			d /= xx;
		}
		if(d > error){
			error = d;
		}
		x(j) = xx;
	}

	{

		int contactIndex = 0;
		for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

			double sum = -M(j, j) * x(j);
			for(int k=0; k < size; ++k){
				sum += M(j, k) * x(k);
			}
			double fx0 = (-b(j) - sum) / M(j, j);
			double& fx = x(j);

			++j;

			sum = -M(j, j) * x(j);
			for(int k=0; k < size; ++k){
				sum += M(j, k) * x(k);
			}
			double fy0 = (-b(j) - sum) / M(j, j);
			double& fy = x(j);

			const double fmax = mcpHi(contactIndex);
			const double fmax2 = fmax * fmax;
			const double fmag2 = fx0 * fx0 + fy0 * fy0;

			if(fmag2 > fmax2){
				const double s = fmax / sqrt(fmag2);
				fx0 *= s;
				fy0 *= s;
			}
			
			double d = fabs(fx0 - fx);
			const double afx0 = fabs(fx0);
			if(afx0 > numeric_limits<double>::epsilon()){
				d /= afx0;
			}
			if(d > error){
				error = d;
			}
			d = fabs(fy0 - fy);
			const double afy0 = fabs(fy0);
			if(afy0 > numeric_limits<double>::epsilon()){
				d /= afy0;
			}
			if(d > error){
				error = d;
			}
			fx = fx0;
			fy = fy0;
		}

	} 

	return error;
}
