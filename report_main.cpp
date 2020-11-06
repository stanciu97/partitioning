#include "src/NodesDistance.h"
#include "src/Spatial.h"
#include "src/Spatial3d.h"
#include <thread>
#include <atomic>
#include "src/SpatioTemporal.h"
#include "src/GeneticEvolution.h"
#include "src/Voronoi.h"
#include "src/OrTools.h"
#include "src/KMedoid.h"
#include <iostream>
#include <chrono> 
#include <filesystem>
#include <fstream>

const std::string input_folder = "input/";
//const std::string output_folder = "report/";

void global(OrTools& solver, std::vector<int>& part, std::atomic<double>& cost, std::atomic<int>& vehicles, bool& acceptable)
{
	auto sol = solver.solve_sub_problem(part);
	if (sol.total_cost == 0)
		acceptable = false;
	cost = cost + sol.total_cost;
	vehicles += sol.number_vehicles;
}

int voronoi_solve_concurrent(nodes &node, NodesDistance& distance, int n_part, std::ofstream& output, bool balanced)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	Voronoi voro(node, distance, balanced);
	auto part = voro.voronoi_part_bubble(n_part);
	std::vector<std::thread> threads;
	std::atomic<double> cost(0);
	std::atomic<int> vehicles(0);
	bool acceptable = true;
	for (auto i = 0; i < part.size(); i++)
	{
		threads.push_back(std::thread(global, std::ref(solver), std::ref(part[i]), std::ref(cost), std::ref(vehicles), std::ref(acceptable)));
	}

	for (auto& th : threads)
		th.join();

	if (!acceptable)
		return -1;
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
	return 0;
}

int voronoi_solve(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output, bool balanced)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	Voronoi voro(node, distance, balanced);
	auto part = voro.voronoi_part_bubble(n_part);
	double cost = 0;
	double vehicles = 0;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		cost += sol.total_cost;
		if (sol.total_cost == 0)
			return -1;
		vehicles += sol.number_vehicles;
	}
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
	return 0;
}

int genetic_solve(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	GeneticEvolution genetic(distance);
	auto part = genetic.genetic_part(n_part);
	double cost = 0;
	double vehicles = 0;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		cost += sol.total_cost;
		if (sol.total_cost == 0)
			return -1;
		vehicles += sol.number_vehicles;
	}
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
	return 0;
}

int genetic_solve_concurrent(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	GeneticEvolution genetic(distance);
	auto part = genetic.genetic_part(n_part);
	std::vector<std::thread> threads;
	std::atomic<double> cost(0);
	std::atomic<int> vehicles(0);
	bool acceptable = true;
	for (auto i = 0; i < part.size(); i++)
	{
		threads.push_back(std::thread(global, std::ref(solver), std::ref(part[i]), std::ref(cost), std::ref(vehicles), std::ref(acceptable)));
	}

	for (auto& th : threads)
		th.join();

	if (!acceptable)
		return -1;
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
	return 0;
}

int medoid_solve(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	KMedoid medoid(distance);
	auto part = medoid.medoid_part(n_part);
	double cost = 0;
	double vehicles = 0;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		cost += sol.total_cost;
		if (sol.total_cost == 0)
			return -1;
		vehicles += sol.number_vehicles;
	}
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
	return 0;
}

int medoid_solve_concurrent(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	KMedoid medoid(distance);
	auto part = medoid.medoid_part(n_part);
	std::vector<std::thread> threads;
	std::atomic<double> cost(0);
	std::atomic<int> vehicles(0);
	bool acceptable = true;
	for (auto i = 0; i < part.size(); i++)
	{
		threads.push_back(std::thread(global, std::ref(solver), std::ref(part[i]), std::ref(cost), std::ref(vehicles), std::ref(acceptable)));
	}

	for (auto& th : threads)
		th.join();

	if (!acceptable)
		return -1;
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
	return 0;
}

void direct_solve(nodes& node, NodesDistance& distance, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	auto sol = solver.solve_problem();
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << sol.total_cost << "    vehicles: " << sol.number_vehicles << "    elapsed time: " << elapsed << std::endl;
}

void solve_from_voronoi(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	Voronoi voro(node, distance);
	auto part = voro.voronoi_part(n_part);
	compact_solution solution;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		solution.routes.insert(solution.routes.end(), sol.routes.begin(), sol.routes.end());
	}
	auto sol = solver.solve_problem_solution(solution.routes);
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << sol.total_cost << "    vehicles: " << sol.number_vehicles << "    elapsed time: " << elapsed << std::endl;
}

void solve_from_genetic(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	GeneticEvolution genetic(distance);
	auto part = genetic.genetic_part(n_part);
	compact_solution solution;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		solution.routes.insert(solution.routes.end(), sol.routes.begin(), sol.routes.end());
	}
	auto sol = solver.solve_problem_solution(solution.routes);
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << sol.total_cost << "    vehicles: " << sol.number_vehicles << "    elapsed time: " << elapsed << std::endl;
}

void solve_from_medoid(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	KMedoid medoid(distance);
	auto part = medoid.medoid_part(n_part);
	compact_solution solution;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		solution.routes.insert(solution.routes.end(), sol.routes.begin(), sol.routes.end());
	}
	auto sol = solver.solve_problem_solution(solution.routes);
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << sol.total_cost << "    vehicles: " << sol.number_vehicles << "    elapsed time: " << elapsed << std::endl;
}


int main()
{
	std::vector<std::string> output_folder = { "report2/", "report5/", "report10/" };
	std::vector<int> n_part = { 2, 5, 10 };
	std::vector<std::string> filename; 
	int n_iter = 5;

	//filenames in the input_folder
	for (const auto& entry : std::filesystem::directory_iterator(input_folder))
	{
		filename.push_back(entry.path().filename().string());
	}

	for (int n = 0; n < n_part.size(); n++)
	{
		for (auto file = 0; file < filename.size(); file++)
		{
			std::ofstream report(output_folder[n] + "Sreport" + filename[file], std::ofstream::out | std::ofstream::trunc);
			nodes node;
			init_nodes(node, input_folder + filename[file]);
			Spatial spatial(node);

			report << "solve with genetic partition:" << std::endl;
			for (auto i = 0; i < n_iter; i++)
			{
				i += genetic_solve_concurrent(node, spatial, n_part[n], report);
			}
			report << std::endl;

			report << "solve with K-medoid partition:" << std::endl;
			for (auto i = 0; i < n_iter; i++)
			{
				i += medoid_solve_concurrent(node, spatial, n_part[n], report);
			}
			report << std::endl;

			report << "solve with balanced voronoi partition:" << std::endl;
			for (auto i = 0; i < n_iter; i++)
			{
				i += voronoi_solve_concurrent(node, spatial, n_part[n], report, true);
			}
			report << std::endl;

			report << "solve with strongest voronoi partition:" << std::endl;
			for (auto i = 0; i < n_iter; i++)
			{
				i += voronoi_solve_concurrent(node, spatial, n_part[n], report, false);
			}
			report << std::endl;

			report.close();

		}
	}
	return 0;

}