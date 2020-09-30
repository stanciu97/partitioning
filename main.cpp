#include "src/NodesDistance.h"
#include "src/Spatial.h"
#include "src/SpatioTemporal.h"
#include "src/GeneticEvolution.h"
#include "src/Voronoi.h"
#include "src/OrTools.h"
#include "src/KMedoid.h"
#include <iostream>
#include <chrono> 

const std::string input_folder = "input/";
const std::string output_folder = "output/";

int main()
{
	int option = -1;

	while (option != 0)
	{
		nodes node;
		std::cout << "Insert input file name ( as name.extension ): ";
		std::string input_name;
		std::cin >> input_name;
		init_nodes(node, input_folder + input_name);

		if (node.id.size() <= 0)
		{
			option = -1;
			std::cout << "file name is not correct" << std::endl;
		}

		else
		{
			compact_solution solution;
			bool found_solution = false;

			std::cout << "insert:" << std::endl <<
				"1: solve problem after partitioning in sub-problems" << std::endl <<
				"2: directly solve problem" << std::endl <<
				"0: end the program" << std::endl;
			std::cin >> option;

			if (option == 1)
			{
				int sub_option;
				NodesDistance* distance = nullptr;
				std::cout << "insert:" << std::endl <<
					"1: use Euclidian distance" << std::endl <<
					"2: use spatiotemporal distance" << std::endl;
				std::cin >> sub_option;

				switch (sub_option)
				{
				case 1:
					distance = new Spatial(node);
					break;
				case 2:
					distance = new SpatioTemporal(node);
					break;
				}

				std::cout << "insert:" << std::endl <<
					"1: use genetic partition" << std::endl <<
					"2: use voronoi partition" << std::endl <<
					"3: use K-Medoid partition" << std::endl;
				std::cin >> sub_option;

				int cluster;
				std::cout << "insert number of desired clusters: ";
				std::cin >> cluster;

				std::vector<std::vector<int>> partition;

				switch (sub_option)
				{
				case 1:
				{
					GeneticEvolution part(*distance);
					partition = part.genetic_part(cluster);
					break;
				}
				case 2:
				{
					partition = iterative_voronoi_part(node, cluster, *distance);
					break;
				}
				case 3:
				{
					KMedoid part(*distance);
					partition = part.medoid_part(cluster);
					break;
				}
				}

				OrTools solver(node);

				for (int i = 0; i < partition.size(); i++)
				{
					auto part_sol = solver.solve_sub_problem(partition[i]);

					//append new partial solution to the actual solution
					solution.routes.insert(solution.routes.end(), part_sol.routes.begin(), part_sol.routes.end());
					solution.number_vehicles += part_sol.number_vehicles;
					solution.route_cost.insert(solution.route_cost.begin(), part_sol.route_cost.begin(), part_sol.route_cost.end());
					solution.total_cost += part_sol.total_cost;
				}

				found_solution = true;
				delete distance;
			}

			else if (option == 2)
			{
				OrTools solver(node);
				solution = solver.solve_problem();
				found_solution = true;
			}

			if (found_solution)
			{
				std::cout << "Insert output file name ( as name.extension ): ";
				std::string output_name;
				std::cin >> output_name;

				OrTools::write_solution(solution, output_folder + output_name);
			}
		}
	}

	return 0;
}