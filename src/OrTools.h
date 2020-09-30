#pragma once
#include "NodesDistance.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

/**
* 
* contains information about a VRPTW solution.
* 
* routes: sequence of customers' id of each route
* route_cost: cost of each route
* total_cost: sum of all routes cost
* number_vehicles: number of vehicles used in the solution
* 
*/
struct compact_solution
{
	std::vector<std::vector<int>> routes;
	std::vector<double> route_cost;
	double total_cost = 0;
	int number_vehicles = 0;
};

/**
* 
* class that uses Google Or-Tools to resove VRPTW instances
* If full problem data are provided also sub-problems can be resolved
* 
*/
class OrTools
{
public:

	/**
	* costructors
	* 
	* input:
	* file: contains a file name as "name.extension", initialize a struct node from file 
	*  or
	* node: reference to an already initialized struct nodes
	* 
	*/
	OrTools(std::string file);
	OrTools(nodes& node);

	/*
	* solve problem based on the struct nodes initialized during construction
	* 
	* output:
	* struct compact_solution containing problem's solution
	* 
	*/
	compact_solution solve_problem();

	/*
	* solve the problem starting from the given solution
	* 
	* input:
	* routes: reference to a vector containing routes as sequence of customers' ids
	* 
	* output:
	* struct compact_solution containing problem's solution
	*
	*/
	compact_solution solve_problem_solution(std::vector<std::vector<int>>& routes);

	/*
	* solve a sub-problem
	*
	* input:
	* sub_id: contains all customers' ids that define the sub-problem 
	*
	* output:
	* struct compact_solution containing sub-problem's solution
	*
	*/
	compact_solution solve_sub_problem(std::vector<int>& sub_id);

	/**
	* write solution to file
	* 
	* input:
	* solution: reference to a problem or subproblem solution
	* name: file name as "name.extension", overrides files with the same name
	* 
	*/
	void static write_solution(compact_solution& solution, std::string name);
	
private:
	nodes node;

	//square matrix that contains travel time between customers. If vehicles speed is considered to be 1 and constant, travel time is equal to the distance
	std::vector<std::vector<int>> time_matrix;

	void init_time_matrix();

	//depot's index is always 0 
	const operations_research::RoutingIndexManager::NodeIndex depot{ 0 };

	void set_constraints(std::vector<std::vector<int>>& time_matrix, nodes& node, operations_research::RoutingIndexManager& manager, operations_research::RoutingModel& routing);

	//summarize the solution given by the solver in a struct compact_solution
	compact_solution readable_solution(nodes& node, const operations_research::RoutingIndexManager& manager, const operations_research::RoutingModel& routing, const operations_research::Assignment& solution);
};