#ifndef CMA_WRAP_H
#define CMA_WRAP_H

extern "C"
{
#include "cmaes_interface.h"
}
class CMAwrap
{
 public:
	cmaes_t evo;/* an CMA-ES type struct or "object" */
	double *arFunvals, *const*pop;

	CMAwrap(vectorn const& start_p, vectorn const& stdev, int populationSize, int mu=-1);
	~CMAwrap();
	std::string testForTermination();	
	void samplePopulation();
	int numPopulation();
	int dim();
	vectornView getPopulation(int i);
	void setVal(int i, double eval);
	void resampleSingle(int i);
	void resampleSingleFrom(int i, vectorn const& v);
	void update();

	void getMean(vectorn& out);
	void getBest(vectorn& out);

	//ys
	double getBestFvEver()			{ return cmaes_Get(&evo, "fbestever"); }
	double getBestFvRecent()		{ return cmaes_Get(&evo, "funvalue"); }
	void getBestSolEver(vectorn& out)	{ getBest(out); }
	void getBestSolRecent(vectorn& out);
};
#endif
