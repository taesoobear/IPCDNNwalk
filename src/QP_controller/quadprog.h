#ifndef _TAESOO_QUADSOLVE_HPP_
#define _TAESOO_QUADSOLVE_HPP_

/*
 NOTE: this is a wrapper of Eigen QuadProg++ package, working with Eigen data structures. 

 The quadprog_solve() function implements the algorithm of Goldfarb and Idnani 
 for the solution of a (convex) Quadratic Programming problem
by means of a dual method.
	 
The problem is in the form:

min 0.5 * x G x + g0 x
s.t.
    CE x + ce0 = 0
    CI x + ci0 >= 0
	 
 The matrix and vectors dimensions are as follows:
     G: n * n
		g0: n
				
		CE: n * p
	 ce0: p
				
	  CI: n * m
   ci0: m

     x: n
 
 The function will return the cost of the solution written in the x vector or
 std::numeric_limits::infinity() if the problem is infeasible. In the latter case
 the value of the x vector is not correct.
 
 References: D. Goldfarb, A. Idnani. A numerically stable dual method for solving
             strictly convex quadratic programs. Mathematical Programming 27 (1983) pp. 1-33.

 Notes:
  1. pay attention in setting up the vectors ce0 and ci0. 
	   If the constraints of your problem are specified in the form 
	   A x = b and C x >= d, then you should set ce0 = -b and ci0 = -d.
    
*/

/* solve_quadprog is used for on-demand QP solving */
double solve_quadprog(const matrixn & G,  const vectorn & g0,  
                      const matrixn & CE, const vectorn & ce0,  
                      const matrixn & CI, const vectorn & ci0, 
                      vectorn & x,bool g0_negative=false);

double solve_quadprog_using_qpOASES(const matrixn & G,  const vectorn & g0,  
                      const matrixn & CE, const vectorn & ce0,  
                      const matrixn & CI, const vectorn & ci0, 
                      vectorn & x, bool g0_negative);

class HessianQuadratic;
double solve_quadprog(HessianQuadratic & problem,
                      const matrixn & CE, const vectorn & ce0,  
                      const matrixn & CI, const vectorn & ci0, 
                      vectorn & x, bool use_qpOASES=false);

/*!
 * @class LCP lcp.h
 * Solves a linear complementarity problem (LCP): find g and a that satisfy
 * N*g + r = a, a>=0, g>=0, a^T*g = 0.
 */
//void solveLCP(const matrixn&  N, const vectorn& r, vectorn& g, vectorn & a);
#endif
