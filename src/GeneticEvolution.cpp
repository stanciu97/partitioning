#include "GeneticEvolution.h"
#include "ConsecutiveRandoms.h"
#include <iostream>
#include <fstream>
#include <set>
#include <array>

GeneticEvolution::GeneticEvolution(NodesDistance& nodes) 
{
	this->nodes = &nodes;
};

GeneticEvolution::GeneticEvolution(NodesDistance& nodes, int p_size, double crossover_p, double crossover_mutation, double candidate_mutation)
{
	this->nodes = &nodes;

	if (p_size > 0)
		this->population_size = p_size;
	if (crossover_p >= 0 && crossover_p <= 1)
		this->crossover_prob = crossover_p;
	if (crossover_mutation >= 0 && crossover_mutation <= 1)
		this->crossover_mutation = crossover_mutation;
	if (candidate_mutation >= 0 && candidate_mutation <= 1)
		this->candidate_mutation = candidate_mutation;
}

std::vector<std::vector<int>> GeneticEvolution::genetic_part(int groups)
{
	return this->genetic_part(groups, this->number_of_generations);
}

std::vector<std::vector<int>> GeneticEvolution::genetic_part(int groups, int n_generations)
{
	//create initial population
	std::vector<std::vector<int>>* population = new std::vector<std::vector<int>>(this->population_size, std::vector<int>(groups, 0));
	std::vector<double> raw_fitness(population_size, 0.0);

	double best_fitness_value = 0;
	std::vector<int> current_best_solution;
	int best_index = 0;

	ConsecutiveRandoms<int> rand(1, this->nodes->get_size() - 1);

	//initialize population using a support set. It assures that elements in the solution are different 
	for (auto i = 0; i < this->population_size; i++)
	{
		std::set<int> temp;
		for (auto j = 0; j < groups; )
		{
			auto medoid = rand.generate();
			temp.insert(medoid);
			if (temp.size() > j)
			{
				population->at(i).at(j) = medoid;
				j++;
			}
		}

		//find best first solution
		raw_fitness[i] = fitness_value(population->at(i));

		if (i == 0)
		{
			best_fitness_value = raw_fitness[i];
			best_index = i;
		}
		else if (raw_fitness[i] < best_fitness_value)
		{
			best_fitness_value = raw_fitness[i];
			best_index = i;
		}
	}

	current_best_solution = population->at(best_index);

	//evolve population
	for (auto gen = 0; gen < n_generations; gen++)
	{
		auto temp_population = new std::vector<std::vector<int>>(this->population_size, std::vector<int>(groups, 0));

		roulette_selection(*temp_population, population, raw_fitness);

		crossover(*temp_population);

		mutation(*temp_population);

		delete population;
		population = temp_population;

		double old_fitness_value = best_fitness_value;

		//look for better candidate solution
		for (auto i = 0; i < population->size(); i++)
		{
			raw_fitness[i] = fitness_value(population->at(i));

			if (raw_fitness[i] < best_fitness_value)
			{
				best_fitness_value = raw_fitness[i];
				best_index = i;
			}
		}

		//update best solution
		if (best_fitness_value < old_fitness_value)
			current_best_solution = population->at(best_index);

	}

	//create groups of the partition
	std::vector<std::vector<int>> genetic_part(groups, std::vector<int>());

	//exclude depot from each group starting with i = 1
	for (auto i = 1; i < this->nodes->get_size(); i++)
	{
		int medoid = 0;

		//assign each customer to the nearest medoid. Assume that the distance from first medoid is the best. Doing so makes easier to update the best value
		double min = this->nodes->get_distance(current_best_solution[0], i);

		for (auto j = 1; j < current_best_solution.size(); j++)
		{
			auto distance = this->nodes->get_distance(current_best_solution[j], i);
			if (distance < min)
			{
				min = distance;
				medoid = j;
			}
		}

		genetic_part[medoid].push_back(i);
	}

	return genetic_part;

}

double GeneticEvolution::fitness_value(std::vector<int> medoids)
{
	double fitness = 0.0;

	//fitness value based on sum of distances of eache customer from its medoid
	for (auto i = 1; i < this->nodes->get_size(); i++)
	{
		double temp_min = this->nodes->get_distance(medoids[0], i);
		for (int j = 1; j < medoids.size(); j++)
		{
			temp_min = std::min(temp_min, this->nodes->get_distance(medoids[j], i));
		}

		fitness += temp_min;
	}

	return fitness;
}

void GeneticEvolution::roulette_selection(std::vector<std::vector<int>>& temp, std::vector<std::vector<int>>* population, std::vector<double>& raw)
{
	std::vector<double> refined_fitness(raw.size(), 0.0);
	double sum = 0.0;

	// 1 / fitness_value
	for (auto i = 0; i < raw.size(); i++)
	{
		refined_fitness[i] = 1.0 / raw[i];
		sum += refined_fitness[i];
	}

	std::vector<double> roulette_slot(refined_fitness.size(), 0.0);
	double total_slot = 0.0;

	//assign to each customer its slot based on the fitness value
	for (auto i = 0; i < roulette_slot.size(); i++)
	{
		auto scaled_fitness = refined_fitness[i] / sum;
		roulette_slot[i] = total_slot + scaled_fitness;
		total_slot += scaled_fitness;
	}

	ConsecutiveRandoms<double> probability(0.0, 1.0);

	for (auto i = 0; i < temp.size(); i++)
	{
		//number in [0;1)
		auto candidate = probability.generate();

		//binary search setup
		int left = 0;
		int right = roulette_slot.size() - 1;
		bool hit = false;

		//binary search adapted for intervals, find customer associated to the slot identified by candidate
		while (!hit)
		{
			int mid = (left + right) / 2;
			if ((mid - 1 >= 0 && candidate >= roulette_slot[mid - 1] && candidate < roulette_slot[mid]) || (mid - 1 < 0 && candidate >= 0.0 && candidate < roulette_slot[mid]))
			{
				temp[i] = population->at(mid);
				hit = true;
			}

			if (candidate > roulette_slot[mid])
			{
				left = mid + 1;
			}
			else
			{
				right = mid - 1;
			}
		}
	}

}

void GeneticEvolution::crossover(std::vector<std::vector<int>>& temp)
{
	ConsecutiveRandoms<int> new_position(0, temp.size() - 1);
	std::vector<int> swapped_position;
	swapped_position.reserve(temp.size());


	for (auto i = 0; i < temp.size(); i++)
	{
		swapped_position.push_back(i);
	}

	//scramble population using the ausiliar swapped_position vector. It avoids the copy of solutions.
	for (auto i = 0; i < temp.size(); i++)
	{
		auto actual = swapped_position[i];
		auto swap = new_position.generate();
		swapped_position[i] = swapped_position[swap];
		swapped_position[swap] = actual;
	}

	ConsecutiveRandoms<double> do_mutation(0.0, 1.0);

	//in case of populations made of odd number of solutions, the last one is simply passed in the next generation because has no partner for recombination
	int even_round = temp.size();

	if (even_round % 2 == 1)
		even_round--;

	for (auto i = 0; i < even_round; i += 2)
	{
		if (do_mutation.generate() <= this->crossover_prob)
			recombine(temp[swapped_position[i]], temp[swapped_position[i + 1]]);
	}

}

void GeneticEvolution::recombine(std::vector<int>& parent1, std::vector<int>& parent2)
{
	//initialize two sets containing the parents
	std::set<int> p1(parent1.begin(), parent1.end());
	std::set<int> p2(parent2.begin(), parent2.end());

	//child is parent1|parent2
	std::vector<int> child;
	child.reserve(parent1.size() + parent2.size());
	child.insert(child.end(), parent1.begin(), parent1.end());
	child.insert(child.end(), parent2.begin(), parent2.end());

	ConsecutiveRandoms<int> new_pos(0, child.size() - 1);

	auto scramble = [&child, &new_pos]()
	{
		for (auto i = 0; i < child.size(); i++)
		{
			auto swap = new_pos.generate();
			auto actual = child[i];
			child[i] = child[swap];
			child[swap] = actual;
		}
	};

	scramble();

	ConsecutiveRandoms<int> rand_candidate(1, this->nodes->get_size() - 1);
	ConsecutiveRandoms<double> rand_mutation(0.0, 1.0);

	/*mutate first i customer in child vector. A mutation is valid if in the vector the same customer appears no more than twice.
	* p1 and p2 contain the parents so are valid solutions, if a customer can not be inserted in neither of the sets it means the
	* customer already appears twice in the child vector. A new customer must be generated
	*/
	for (auto i = 0; i < parent1.size(); i++)
	{
		auto got_mutation = rand_mutation.generate();

		if (got_mutation <= this->crossover_mutation)
		{
			bool good_candidate = false;
			int candidate;
			while (!good_candidate)
			{
				candidate = rand_candidate.generate();

				if (p1.find(candidate) == p1.end())
				{
					p1.insert(candidate);
					good_candidate = true;
				}

				else if (p2.find(candidate) == p2.end())
				{
					p2.insert(candidate);
					good_candidate = true;
				}

			}
			child[i] = candidate;
		}
	}

	scramble();

	int index1 = 0, index2 = 0, child_index = 0;

	std::set<int> child1;
	std::set<int> child2;

	//generate child1 scrolling child vector from left to right avoiding douplicates
	while (index1 < parent1.size())
	{
		if (child1.find(child[child_index]) == child1.end())
		{
			child1.insert(child[child_index]);
			parent1[index1] = child[child_index];
			index1++;
		}

		child_index++;
	}

	child_index = child.size() - 1;

	//generate child2 scrolling child vector from right to left avoiding douplicates
	while (index2 < parent2.size())
	{
		if (child2.find(child[child_index]) == child2.end())
		{
			child2.insert(child[child_index]);
			parent2[index2] = child[child_index];
			index2++;
		}

		child_index--;
	}

}

void GeneticEvolution::mutation(std::vector<std::vector<int>>& temp)
{
	//mutate a singol customer in the solution. A set is used to check if the mutaion is valid
	auto mute = [this](std::vector<int>& chromosome)
	{
		std::set<int> chrm_set(chromosome.begin(), chromosome.end());

		int position = ConsecutiveRandoms<int>::generate(0, chromosome.size() - 1);
		bool mutated = false;

		while (!mutated)
		{
			int candidate = ConsecutiveRandoms<int>::generate(1, this->nodes->get_size() - 1);

			if (chrm_set.find(candidate) == chrm_set.end())
			{
				chromosome[position] = candidate;
				mutated = true;
			}
		}
	};

	ConsecutiveRandoms<double> do_mutation(0.0, 1.0);

	for (auto i = 0; i < temp.size(); i++)
	{
		if (do_mutation.generate() <= this->candidate_mutation)
		{
			mute(temp[i]);
		}
	}
}