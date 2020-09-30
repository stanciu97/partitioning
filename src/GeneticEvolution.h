#pragma once
#include "NodesDistance.h"

/*
* class that implements a genetic algorithm based on 
* K-medoid partitioning. It aims to partion a large
* VRPTW problem in various sub-problems which are solvable easier then the 
* orginal one.
* 
*/

class GeneticEvolution
{
public:
	/**
	* costructors
	* 
	* input:
	* nodes: reference to an istance of NodeDistance, determines the distance used in the algorithm (Euclidean, spatiotemporal)
	* p_size: size of the population
	* crossover_p: probability that during a crossover parents generates offsprings different from them
	* crossover_mutation: probability that a mutation occurs during crossover
	* candidate_mutation: probabilty that a mutation occurs at the crossover's end
	* 
	*/
	GeneticEvolution(NodesDistance& nodes);
	GeneticEvolution(NodesDistance& nodes, int p_size, double crossover_p, double crossover_mutation, double candidate_mutation);

	/**
	* function that creates partition after finding the best candidate during the given number of generations
	* 
	* input:
	* groups: total number of clusters in the partition
	* n_generation: number of generations evolved
	* 
	* output:
	* best found partition after n_generation
	* 
	*/
	std::vector<std::vector<int>> genetic_part(int groups);
	std::vector<std::vector<int>> genetic_part(int groups, int n_generations);

private:
	double fitness_value(std::vector<int> medoids);
	void roulette_selection(std::vector<std::vector<int>>& temp, std::vector<std::vector<int>>* population, std::vector<double>& raw);
	void crossover(std::vector<std::vector<int>>& temp);
	void recombine(std::vector<int>& parent1, std::vector<int>& parent2);
	void mutation(std::vector<std::vector<int>>& temp);
	NodesDistance* nodes;

	//default genetic parameters
	int number_of_generations = 300;
	int population_size = 100;
	double crossover_prob = 0.65;
	double crossover_mutation = 0.2;
	double candidate_mutation = 0.05;
};
