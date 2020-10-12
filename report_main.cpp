#include "src/NodesDistance.h"
#include "src/Spatial.h"
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
const std::string output_folder = "report/";

void voronoi_solve(nodes &node, NodesDistance& distance, int n_part, std::ofstream& output)
{
	OrTools solver(node);
	auto clock_start = std::chrono::system_clock::now();
	Voronoi voro(node, distance);
	auto part = voro.voronoi_part(n_part);
	double cost = 0;
	double vehicles = 0;
	for (auto i = 0; i < part.size(); i++)
	{
		auto sol = solver.solve_sub_problem(part[i]);
		cost += sol.total_cost;
		vehicles += sol.number_vehicles;
	}
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
}

void genetic_solve(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
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
		vehicles += sol.number_vehicles;
	}
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
}

void medoid_solve(nodes& node, NodesDistance& distance, int n_part, std::ofstream& output)
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
		vehicles += sol.number_vehicles;
	}
	auto clock_end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast <std::chrono::seconds> (clock_end - clock_start).count();
	output << "cost: " << cost << "    vehicles: " << vehicles << "    elapsed time: " << elapsed << std::endl;
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
	std::vector<std::string> filename; 
	int n_iter = 3;
	int n_part = 10;

	//filenames in the input_folder
	for (const auto& entry : std::filesystem::directory_iterator(input_folder))
	{
		filename.push_back(entry.path().filename().string());
	}

	for (auto file = 0; file < filename.size(); file++)
	{
		std::ofstream report(output_folder + "STreport" + filename[file], std::ofstream::out | std::ofstream::trunc);
		nodes node;
		init_nodes(node, input_folder + filename[file]);
		SpatioTemporal spatio_temporal(node);

		/*report << "solve without partition:" << std::endl;
		for (auto i = 0; i < 3; i++)
		{
			direct_solve(node, spatio_temporal, report);
		}
		report << std::endl;*/

		report << "solve with solution from genetic partition:" << std::endl;
		for (auto i = 0; i < n_iter; i++)
		{
			solve_from_genetic(node, spatio_temporal, n_part, report);
		}
		report << std::endl;

		report << "solve with solution from K-medoid partition:" << std::endl;
		for (auto i = 0; i < n_iter; i++)
		{
			solve_from_medoid(node, spatio_temporal, n_part, report);
		}
		report << std::endl;

		report << "solve with solution from voronoi partition:" << std::endl;
		for (auto i = 0; i < n_iter; i++)
		{
			solve_from_voronoi(node, spatio_temporal, n_part, report);
		}
		report << std::endl;

		report.close();
		
	}

	for (auto file = 0; file < filename.size(); file++)
	{
		std::ofstream report(output_folder + "Sreport" + filename[file], std::ofstream::out | std::ofstream::trunc);
		nodes node;
		init_nodes(node, input_folder + filename[file]);
		Spatial spatial(node);

		/*report << "solve without partition:" << std::endl;
		for (auto i = 0; i < 3; i++)
		{
			direct_solve(node, spatial, report);
		}
		report << std::endl;*/

		report << "solve with solution from genetic partition:" << std::endl;
		for (auto i = 0; i < n_iter; i++)
		{
			solve_from_genetic(node, spatial, n_part, report);
		}
		report << std::endl;

		report << "solve with solution from K-medoid partition:" << std::endl;
		for (auto i = 0; i < n_iter; i++)
		{
			solve_from_medoid(node, spatial, n_part, report);
		}
		report << std::endl;

		report << "solve with solution from voronoi partition:" << std::endl;
		for (auto i = 0; i < n_iter; i++)
		{
			solve_from_voronoi(node, spatial, n_part, report);
		}
		report << std::endl;

		report.close();

	}

	return 0;

}