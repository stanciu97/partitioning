#include "KMedoid.h"
#include "ConsecutiveRandoms.h"
#include <set>
#include <iostream>

KMedoid::KMedoid(NodesDistance& nodes)
{
	this->nodes = &nodes;
}

std::vector<std::vector<int>> KMedoid::medoid_part(int groups)
{
	return this->medoid_part(groups, this->iterations);
}

std::vector<std::vector<int>> KMedoid::medoid_part(int groups, int n_iter)
{
	std::vector<int> solution_medoid;
	double partition_cost = 0;

	//make n_iter attempts and take best group of medoids
	for (int attempt = 0; attempt < n_iter; attempt++)
	{
		ConsecutiveRandoms<int> rand_medoid(1, this->nodes->get_size() - 1);
		std::set<int> temp_medoid;
		std::vector<int> medoid(groups);

		//generate random seeds
		for (auto i = 0; i < medoid.size(); )
		{
			auto candidate = rand_medoid.generate();
			temp_medoid.insert(candidate);
			if (temp_medoid.size() > i)
			{
				medoid[i] = candidate;
				i++;
			}
		}

		bool changed_medoids = true;

		while (changed_medoids)
		{
			auto temp_cost = 0;
			changed_medoids = false;
			std::vector<std::vector<int>> actual_partition(groups, std::vector<int>());
			std::vector<double> medoid_cost(groups, 0);

			//reserve space for each group, insert medoids
			int medium_size = (this->nodes->get_size() - 1) / groups;
			for (auto i = 0; i < medoid.size(); i++)
			{
				actual_partition[i].reserve(medium_size);
			}

			//assign customers to the nearer group
			for (auto i = 1; i < this->nodes->get_size(); i++)
			{
				double min_distance = this->nodes->get_distance(medoid[0], i);
				int best_medoid = 0;
				for (int j = 1; j < medoid.size(); j++)
				{
					auto temp_distance = this->nodes->get_distance(medoid[j], i);
					if (temp_distance < min_distance)
					{
						min_distance = temp_distance;
						best_medoid = j;
					}
				}

				actual_partition[best_medoid].push_back(i);
				medoid_cost[best_medoid] += min_distance;
			}

			//look for better medoids
			for (auto i = 0; i < medoid.size(); i++)
			{
				for (auto j = 1; j < actual_partition[i].size(); j++)
				{
					auto candidate = actual_partition[i][j];
					double candidate_group_distance = 0;

					for (auto k = 0; k < actual_partition[i].size(); k++)
					{
						candidate_group_distance += this->nodes->get_distance(candidate, actual_partition[i][k]);
					}

					//update medoid if there is a better gravity point
					if (candidate_group_distance < medoid_cost[i])
					{
						medoid[i] = candidate;
						medoid_cost[i] = candidate_group_distance;
						changed_medoids = true;
					}
				}
				temp_cost += medoid_cost[i];
			}

			if (attempt == 0)
			{
				solution_medoid = medoid;
				partition_cost = temp_cost;
			}

			else if (temp_cost < partition_cost)
			{
				solution_medoid = medoid;
				partition_cost = temp_cost;
			}
		}
	}

	//create partition from medoids with lowest cost
	std::vector<std::vector<int>> solution_partition(groups, std::vector<int>());
	for (auto i = 1; i < this->nodes->get_size(); i++)
	{
		double min_distance = this->nodes->get_distance(solution_medoid[0], i);
		int best_medoid = 0;
		for (int j = 1; j < solution_medoid.size(); j++)
		{
			auto temp_distance = this->nodes->get_distance(solution_medoid[j], i);
			if (temp_distance < min_distance)
			{
				min_distance = temp_distance;
				best_medoid = j;
			}
		}

		solution_partition[best_medoid].push_back(i);
	}

	return solution_partition;
}