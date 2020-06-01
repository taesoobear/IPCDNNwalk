#ifndef GA_SIMPLE_H_
#define GA_SIMPLE_H_
#include <BaseLib/baselib.h>

class Individual {
		// Cache
		mutable double fitness;
	public:
		boolN genes;

    static int defaultGeneLength ;
    // Use this if you want to create individuals with different gene lengths
    static void setDefaultGeneLength(int length) { defaultGeneLength = length; }
    static void setDefaultTrueGeneRate(double rate);

	Individual();
    int size() const { return genes.size();}
    virtual double getFitness() const ;

    void setGene(int index, bool value) { genes.set(index, value); fitness = 0; }
    bool getGene(int index) const { return genes[index]; }
	std::string toString() const;
};
class Population {
		std::vector<Individual> individuals;
	public:
		
    // Create a population
    Population(int populationSize) ;

    /* Getters */
    Individual & getIndividual(int index) { return individuals[index]; }
    Individual const & getIndividual(int index) const { return individuals[index]; }
    // Get population size
    int size() const { return individuals.size(); }
    // Save individual
    void saveIndividual(int index, Individual const& indiv) { individuals[index] = indiv; }

	const Individual & getFittest() const;
};
class FitnessCalc {
	public:
    virtual double getFitness(Individual const& individual);
    virtual double getMaxFitness();
};
void GA_setParam(double mutationRate, int tournamentSize);
Population* GA_solve(int numPop, int maxGenerations);
void setFitness(FitnessCalc* pcalc);
#endif
