#include "GA_simple.h"
#include <iostream>
using namespace std;
//Creating a genetic algorithm for beginners
//Introduction
//A genetic algorithm (GA) is great for finding solutions to complex search problems. They're often used in fields such as engineering to create incredibly high quality products thanks to their ability to search a through a huge combination of parameters to find the best match. For example, they can search through different combinations of materials and designs to find the perfect combination of both which could result in a stronger, lighter and overall, better final product. They can also be used to design computer algorithms, to schedule tasks, and to solve other optimization problems. Genetic algorithms are based on the process of evolution by natural selection which has been observed in nature. They essentially replicate the way in which life uses evolution to find solutions to real world problems. Surprisingly although genetic algorithms can be used to find solutions to incredibly complicated problems, they are themselves pretty simple to use and understand.
//
//How they work
//As we now know they're based on the process of natural selection, this means they take the fundamental properties of natural selection and apply them to whatever problem it is we're trying to solve.
//
//The basic process for a genetic algorithm is:
//
//    Initialization - Create an initial population. This population is usually randomly generated and can be any desired size, from only a few individuals to thousands.
//    Evaluation - Each member of the population is then evaluated and we calculate a 'fitness' for that individual. The fitness value is calculated by how well it fits with our desired requirements. These requirements could be simple, 'faster algorithms are better', or more complex, 'stronger materials are better but they shouldn't be too heavy'.
//    Selection - We want to be constantly improving our populations overall fitness. Selection helps us to do this by discarding the bad designs and only keeping the best individuals in the population.  There are a few different selection methods but the basic idea is the same, make it more likely that fitter individuals will be selected for our next generation.
//    Crossover - During crossover we create new individuals by combining aspects of our selected individuals. We can think of this as mimicking how sex works in nature. The hope is that by combining certain traits from two or more individuals we will create an even 'fitter' offspring which will inherit the best traits from each of it's parents.
//    Mutation - We need to add a little bit randomness into our populations' genetics otherwise every combination of solutions we can create would be in our initial population. Mutation typically works by making very small changes at random to an individuals genome.
//    And repeat! - Now we have our next generation we can start again from step two until we reach a termination condition.
//
//Termination
//There are a few reasons why you would want to terminate your genetic algorithm from continuing it's search for a solution. The most likely reason is that your algorithm has found a solution which is good enough and meets a predefined minimum criteria. Offer reasons for terminating could be constraints such as time or money.
//
//Limitations
//Imagine you were told to wear a blindfold then you were placed at the bottom of a hill with the instruction to find your way to the peak. You're only option is to set off climbing the hill until you notice you're no longer ascending anymore. At this point you might declare you've found the peak, but how would you know? In this situation because of your blindfolded you couldn't see if you're actually at the peak or just at the peak of smaller section of the hill. We call this a local optimum. Below is an example of how this local optimum might look:
//
//Unlike in our blindfolded hill climber, genetic algorithms can often escape from these local optimums if they are shallow enough. Although like our example we are often never able to guarantee that our genetic algorithm has found the global optimum solution to our problem. For more complex problems it is usually an unreasonable exception to find a global optimum, the best we can do is hope for is a close approximation of the optimal solution.
//
//Implementing a basic binary genetic algorithm in Java
//These examples are build in Java. If you don't have Java installed and you want to follow along please head over to the Java downloads page, http://www.oracle.com/technetwork/java/javase/downloads/index.html
//
//Let's take a look at the classes we're going to create for our GA:
//
//    Population - Manages all individuals of a population
//    Individual - Manages an individuals
//    Algorithm - Manages our evolution algorithms such as crossover and mutation
//    FitnessCalc - Allows us set a candidate solution and calculate an individual's fitness
//

int Individual::defaultGeneLength=64;

static FitnessCalc * g_fitnessCalc=NULL;
static double math_random()
{
	return ((double)rand()/(double)RAND_MAX);
}
static double defaultTrueGeneRate=0.5;
void Individual::setDefaultTrueGeneRate(double rate)
{
	defaultTrueGeneRate=rate;
}
// Create a random individual
Individual::Individual() {
	genes.resize(defaultGeneLength);
	fitness=0;
	for (int i = 0; i < size(); i++) {
		bool gene = math_random()>(1.0-defaultTrueGeneRate);
		genes.set(i, gene);
	}
}

double Individual::getFitness() const{
	if (fitness == 0) {
		fitness = g_fitnessCalc->getFitness(*this);
	}
	return fitness;
}

std::string Individual::toString() const {
	std::string geneString = "";
	for (int i = 0; i < size(); i++) {
		if(getGene(i))
			geneString += "1";
		else
			geneString += "0";
	}
    return geneString;
}


/*
 * Constructors
 */
// Create a population
Population::Population(int populationSize) {
	individuals .resize(populationSize);
}


const Individual & Population::getFittest() const {
        const Individual * fittest = &individuals[0];
        // Loop through individuals to find fittest
        for (int i = 0; i < size(); i++) {
            if (fittest->getFitness() <= getIndividual(i).getFitness()) {
                fittest =& getIndividual(i);
            }
        }
        return *fittest;
    }




namespace Algorithm {

    /* GA parameters */
    static double uniformRate = 0.5;
    static double mutationRate = 0.015;
    static int tournamentSize = 5;
    static bool elitism = true;

    /* Public methods */
    

    // Crossover individuals
    static Individual  crossover(Individual const& indiv1, Individual const& indiv2) {
        Individual newSol ;
        // Loop through genes
        for (int i = 0; i < indiv1.size(); i++) {
            // Crossover
            if (math_random() <= uniformRate) {
                newSol.setGene(i, indiv1.getGene(i));
            } else {
                newSol.setGene(i, indiv2.getGene(i));
            }
        }
        return newSol;
    }

    // Mutate an individual
    static void mutate(Individual &indiv) {
        // Loop through genes
        for (int i = 0; i < indiv.size(); i++) {
            if (math_random() <= mutationRate) {
                // Create random gene
                bool gene = math_random()>0.5;
                indiv.setGene(i, gene);
            }
        }
    }

    // Select individuals for crossover
    static Individual tournamentSelection(Population const& pop) {
        // Create a tournament population
		//
        Population tournament (tournamentSize);
        // For each place in the tournament get a random individual
        for (int i = 0; i < tournamentSize; i++) {
            int randomId = (int) (math_random() * pop.size());
            tournament.saveIndividual(i, pop.getIndividual(randomId));
        }
        // Get the fittest
        return tournament.getFittest();
    }
    // Evolve a population
    Population* evolvePopulation(Population const& pop) {
        Population * newPopulation = new Population(pop.size());

        // Keep our best individual
        if (elitism) {
            newPopulation->saveIndividual(0, pop.getFittest());
        }

        // Crossover population
        int elitismOffset;
        if (elitism) {
            elitismOffset = 1;
        } else {
            elitismOffset = 0;
        }
        // Loop over the population size and create new individuals with
        // crossover
        for (int i = elitismOffset; i < pop.size(); i++) {
            Individual indiv1 = tournamentSelection(pop);
            Individual indiv2 = tournamentSelection(pop);
            Individual newIndiv = crossover(indiv1, indiv2);
            newPopulation->saveIndividual(i, newIndiv);
        }

        // Mutate population
        for (int i = elitismOffset; i < newPopulation->size(); i++) {
            mutate(newPopulation->getIndividual(i));
        }
#if 0 // VERBOSE mode
        for (int i = 0; i < newPopulation->size(); i++) {
			cout <<newPopulation->getIndividual(i).toString()<<endl;
		}
		cout <<"________________"<<endl;
#endif

        return newPopulation;
    }
}

void GA_setParam(double m, int t)
{
	Algorithm::mutationRate=m;
	Algorithm::tournamentSize=t;
}

static boolN solution ;

    // To make it easier we can use this method to set our candidate solution
    // with string of 0s and 1s
    static void setSolution(std::string newSolution) {
        solution .resize(newSolution.size());
        // Loop through each character of our string and save it in our byte
        // array
        for (int i = 0; i < newSolution.size(); i++) {
			if (newSolution[i]=='1')
				solution.setAt(i);
			else
                solution.clearAt(i);
		}
	}

    // Calculate inidividuals fittness by comparing it to our candidate solution
    double FitnessCalc ::getFitness(Individual const& individual) {
        int fitness = 0;
        // Loop through our individuals genes and compare them to our cadidates
        for (int i = 0; i < individual.size() && i < solution.size(); i++) {
            if (individual.getGene(i) == solution[i]) {
                fitness++;
            }
        }
        return (double)fitness;
    }
    
    // Get optimum fitness
    double FitnessCalc ::getMaxFitness() {
        double maxFitness = solution.size();
        return maxFitness;
    }


void GA_main() {
	FitnessCalc f;
	g_fitnessCalc=&f;

	// First we need to set a candidate solution (feel free to change this if you want to).
	// Set a candidate solution
	std::string solution="1111000000000000000000000000000000000000000000000000000000001111";
	setSolution(solution);

	int maxGenerations = 50;
	Population *myPop =GA_solve(50, maxGenerations);

	//cout<<"Solution:"<<endl<<solution <<endl;
	//cout<<"Genes:"<<endl;
	//cout<<myPop->getFittest().toString()<<endl;
	delete myPop;
}

void setFitness(FitnessCalc* pcalc)
{
	g_fitnessCalc=pcalc;
}

Population* GA_solve(int numPop, int maxGenerations)
{
	// Now we'll create our initial population, a population of 50 should be fine.
	// Create an initial population
	Population *myPop =new Population(numPop);

	// Evolve our population until we reach an optimum solution
	int generationCount = 0;

	while (myPop->getFittest().getFitness() < g_fitnessCalc->getMaxFitness() && generationCount<maxGenerations) {
		generationCount++;
		cout <<"Generation: " << generationCount << "/" <<maxGenerations<< " Fittest: " << myPop->getFittest().getFitness()<<endl;
		Population* old=myPop;
		myPop=Algorithm::evolvePopulation(*old);
		delete old;
	}
	cout<<"Generation: " << generationCount<<endl;
	return myPop;

}
//If everything's right, you should get an output similar to the following:
//Generation: 1 Fittest: 40
//Generation: 2 Fittest: 43
//Generation: 3 Fittest: 50
//Generation: 4 Fittest: 50
//Generation: 5 Fittest: 52
//Generation: 6 Fittest: 59
//Generation: 7 Fittest: 59
//Generation: 8 Fittest: 61
//Generation: 9 Fittest: 61
//Generation: 10 Fittest: 61
//Generation: 11 Fittest: 63
//Generation: 12 Fittest: 63
//Generation: 13 Fittest: 63
//Generation: 14 Fittest: 63
//Generation: 15 Fittest: 63
//Solution found!
//Generation: 15
//Genes:
//1111000000000000000000000000000000000000000000000000000000001111
//
//Remember you're output isn't going to be exactly the same as above because of the inherent characteristics of a genetic algorithm.
//
//And there you have it, that's a very basic binary GA. The great thing about a binary GA is that it is easy to represent any problem, although it might not always be the best way of going about it.
//
//Want to apply a genetic algorithm to a real search problem? Check out the following tutorial, applying a genetic algorithm to the traveling salesman problem
//
//Author
//Lee JacobsonHello, I'm Lee.
