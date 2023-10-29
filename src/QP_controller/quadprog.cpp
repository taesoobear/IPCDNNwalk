#ifdef USE_MPI
#include <mpi.h>
#endif
#include <Eigen/Core>
#include "eiquadprog.hpp"
#include "stdafx.h"
#include "../taesooLib/BaseLib/math/OperatorStitch.h"
#include "quadprog.h"
static void assignT(Eigen::MatrixXd& _a, matrixn const& a)
{
	_a.resize(a.cols(), a.rows());
	for(int i=0; i<a.rows(); i++)
		for(int j=0; j<a.cols(); j++)
			_a(j,i)=a(i,j);
}
static void assign(Eigen::MatrixXd& _a, matrixn const& a)
{
	_a.resize(a.rows(), a.cols());
	for(int i=0; i<a.rows(); i++)
		for(int j=0; j<a.cols(); j++)
			_a(i,j)=a(i,j);
}
static void assign(matrixn& _a, Eigen::MatrixXd const& a)
{
	_a.setSize(a.rows(), a.cols());
	for(int i=0; i<a.rows(); i++)
		for(int j=0; j<a.cols(); j++)
			_a(i,j)=a(i,j);
}
static void assign(Eigen::VectorXd& _a, vectorn const& a)
{
	_a.resize(a.size());
	for(int i=0; i<a.size(); i++)
		_a(i)=a(i);
}
static void assign(vectorn & _a, Eigen::VectorXd const& a)
{
	_a.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		_a(i)=a(i);
}
double solve_quadprog(const matrixn & G,  const vectorn & g0,  
                      const matrixn & CE, const vectorn & ce0,  
                      const matrixn & CI, const vectorn & ci0, 
                      vectorn & x, bool g0_negative)
{
	Eigen::MatrixXd _G,_CE,_CI;
	Eigen::VectorXd _g0,_ce0,_ci0,_x;
	assign(_G,G);
	assignT(_CE,CE);
	assignT(_CI,CI);
	assign(_g0,g0);
	if (g0_negative) { _g0=_g0*-1;}
	assign(_ce0,ce0);
	assign(_ci0,ci0);

	Eigen::solve_quadprog(_G,_g0,_CE,_ce0,_CI,_ci0,_x);
	assign(x,_x);
	return 0;
}
double solve_quadprog( HessianQuadratic & problem,
                      const matrixn & CE, const vectorn & ce0,  
                      const matrixn & CI, const vectorn & ci0, 
                      vectorn & x, bool use_qpOASES)
{
	if (use_qpOASES)
		solve_quadprog_using_qpOASES(problem.H,problem.R, CE, ce0, CI, ci0, x, true);
	else
		solve_quadprog(problem.H,problem.R, CE, ce0, CI, ci0, x, true);
	return 0;
}	
