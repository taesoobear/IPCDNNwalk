#include "../stdafx.h"
#include "CMAwrap.h"
#include <string>


CMAwrap::CMAwrap(vectorn const& start_p, vectorn const& stdev, int populationSize, int mu)
{
	arFunvals=cmaes_init(&evo, start_p.size(), start_p.dataPtr(), stdev.dataPtr(), 0, populationSize,mu, "../../BaseLib/math/cma/initials.par");

	printf("%s \n", cmaes_SayHello(&evo));

	cmaes_ReadSignals(&evo, "../../BaseLib/math/cma/signals.par");
}

std::string CMAwrap::testForTermination()
{
	const char *     temp;
	temp=cmaes_TestForTermination(&evo);
	if (temp==NULL)
		return std::string("");
	return std::string(temp);
}

void CMAwrap::samplePopulation()
{
	/* generate lambda new search points, sample population */
	pop = cmaes_SamplePopulation(&evo); /* do not change content of pop */

}
int  CMAwrap::numPopulation()
{
	return (int)cmaes_Get(&evo, "popsize");
}
int  CMAwrap::dim()
{
	return (int)cmaes_Get(&evo, "dim");
}
vectornView CMAwrap::getPopulation(int i)
{
	return vectornView(pop[i], dim(), 1);
}
void CMAwrap::setVal(int i, double eval)
{
	arFunvals[i]=eval;
}
void CMAwrap::resampleSingle(int i)
{
	cmaes_ReSampleSingle(&evo,i);
}
void CMAwrap::resampleSingleFrom(int i, vectorn const& v)
{
	cmaes_ReSampleSingleFrom(&evo, i, (double*)&v(0));
}

void CMAwrap::update()
{
	cmaes_UpdateDistribution(&evo, arFunvals);
	cmaes_ReadSignals(&evo, "../../BaseLib/math/cma/signals.par");

}

void CMAwrap::getMean(vectorn& out)
{
	double *xfinal=cmaes_GetNew(&evo, "xmean");
	out.resize(dim());
	for(int i=0; i<dim(); i++)
		out[i]=xfinal[i];
	free(xfinal);
}
void CMAwrap::getBest(vectorn& out)
{
	double *xfinal=cmaes_GetNew(&evo, "xbestever");
	out.resize(dim());
	for(int i=0; i<dim(); i++)
		out[i]=xfinal[i];
	free(xfinal);
}

//ys
void CMAwrap::getBestSolRecent(vectorn& out)
{
	double *xfinal=cmaes_GetNew(&evo, "xbest");
	out.resize(dim());
	for(int i=0; i<dim(); i++)
		out[i]=xfinal[i];
	free(xfinal);
}
	
CMAwrap::~CMAwrap()
{
	cmaes_exit(&evo);
}
