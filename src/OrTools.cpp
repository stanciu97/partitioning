#include "OrTools.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ConsecutiveRandoms.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>


using namespace operations_research;

void OrTools::write_solution(compact_solution& solution, std::string name)
{
    std::ofstream file(name, std::ofstream::out | std::ofstream::trunc);

    file << "numero di veicoli usati: " << solution.number_vehicles << std::endl;
    file << "costo totale: " << solution.total_cost << std::endl << std::endl;

    for (auto i = 0; i < solution.number_vehicles; i++)
    {
        for (auto j = 0; j < solution.routes[i].size(); j++)
        {
            file << solution.routes[i][j] << " ";
        }
        file << std::endl << std::endl;
    }

    file.close();
}

OrTools::OrTools(std::string file)
{
    init_nodes(this->node, file);
    init_time_matrix();
}

OrTools::OrTools(nodes& node)
{
    this->node = node;
    init_time_matrix();
}

void OrTools::init_time_matrix()
{
    auto size = this->node.id.size();
    this->time_matrix.resize(size);

    //travel time as Euclidian distance. The values ar rounded to the nearer integer
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            this->time_matrix[i].push_back(std::sqrt(std::pow(this->node.coord[i][0] - this->node.coord[j][0], 2) + std::pow(this->node.coord[i][1] - this->node.coord[j][1], 2) + 0.5));
        }
    }
}

compact_solution OrTools::solve_problem()
{
    //create index manager and model used by Google Or-tools
    RoutingIndexManager manager(time_matrix.size(), this->node.vehicles, this->depot);
    RoutingModel routing(manager);
    this->set_constraints(this->time_matrix, this->node, manager, routing);

    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    // Solve the problem.
    const Assignment* solution = routing.SolveWithParameters(searchParameters);

    return this->readable_solution(this->node, manager, routing, *solution);
}

compact_solution OrTools::solve_sub_problem(std::vector<int> &sub_id)
{
    //initialize sub_problem structure
    nodes sub_node;
    init_sub_nodes(this->node, sub_node, sub_id);
    auto size = sub_node.id.size();

    std::vector<std::vector<int>> sub_time_matrix(size, std::vector<int>(size));
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            sub_time_matrix[i][j] = this->time_matrix[sub_node.id[i]][sub_node.id[j]];
        }
    }

    //same as in solve_problem() using sub_problem data
    RoutingIndexManager manager(sub_time_matrix.size(), this->node.vehicles, this->depot);
    RoutingModel routing(manager);
    
    this->set_constraints(sub_time_matrix, sub_node, manager, routing);
    
    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    // Solve the problem.
    const Assignment* solution = routing.SolveWithParameters(searchParameters);

    return this->readable_solution(sub_node, manager, routing, *solution);
}

compact_solution OrTools::solve_problem_solution(std::vector<std::vector<int>> &routes)
{
    RoutingIndexManager manager(this->time_matrix.size(), this->node.vehicles, this->depot);
    RoutingModel routing(manager);
    this->set_constraints(this->time_matrix, this->node, manager, routing);

    //conversion from int to int64. Or Tools can not use the given solution otherwise
    std::vector<std::vector<int64>> converted_routes(routes.size(), std::vector<int64>());
    for (int i = 0; i < routes.size(); i++)
    {
        for (int j = 0; j < routes[i].size(); j++)
        {
            converted_routes[i].push_back(routes[i][j]);
        }
    }

    //generate initial solution based on the given solution
    const Assignment* initial_solution = routing.ReadAssignmentFromRoutes(converted_routes, true);

    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    // Solve the problem using the initial solution
    const Assignment* solution = routing.SolveFromAssignmentWithParameters(initial_solution, searchParameters);

    return this->readable_solution(node, manager, routing, *solution);
}

void OrTools::set_constraints(std::vector<std::vector<int>>& time_matrix, nodes& node, RoutingIndexManager& manager, RoutingModel& routing)
{
    //set arc cost between customers as travel_time + service_time
    const int transit_callback_index = routing.RegisterTransitCallback(
        [&time_matrix, &node, &manager](int64 from_index, int64 to_index) -> int64 {
            auto from_node = manager.IndexToNode(from_index).value();
            auto to_node = manager.IndexToNode(to_index).value();
            return time_matrix[from_node][to_node] + node.service_time[from_node];
        });

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    std::string time{ "Time" };
    routing.AddDimension(transit_callback_index,  // transit callback index
        int64{ 30000000 },                        // allow waiting time (high value means that a vehicle can stay as lons as necessary)
        int64{ 30000000 },                        // maximum time per vehicle (high value means that there is no maximum time)
        false,                                    // Don't force start cumul to zero (vehicles departure is not forced to be at time 0)
        time);
    const RoutingDimension& time_dimension = routing.GetDimensionOrDie(time);
    // Add time window constraints for each location except depot.
    for (int i = 1; i < node.time_window.size(); ++i) {
        int64 index = manager.NodeToIndex(RoutingIndexManager::NodeIndex(i));
        time_dimension.CumulVar(index)->SetRange(node.time_window[i][0],
            node.time_window[i][1]);
    }
    // Add time window constraints for each vehicle start node.
    for (int i = 0; i < node.vehicles; ++i) {
        int64 index = routing.Start(i);
        time_dimension.CumulVar(index)->SetRange(node.time_window[0][0],
            node.time_window[0][1]);
    }
    for (int i = 0; i < node.vehicles; ++i) {
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)));
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)));
    }

    //set demand as customer's demand
    const int demand_callback_index = routing.RegisterUnaryTransitCallback(
        [&node, &manager](int64 from_index) -> int64 {
            int from_node = manager.IndexToNode(from_index).value();
            return node.demand[from_node];
        });

    // Add vehicle capacity constraint
    std::vector<int64> cap(node.vehicles, node.capacity);
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        int64{ 0 },
        cap,
        true,                     // start cumul to zero (in reality vehicles start loaded and they are unloaded during the route.
                                  //Here vehicles are considered empty and for each customer visited the vehicle is loaded by an amout equal to the customer's request)
        "Capacity");

    // Instantiate route start and end times to produce feasible times.
    for (int i = 0; i < node.vehicles; ++i) {
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)));
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)));
    }

}

compact_solution OrTools::readable_solution(nodes& node, const RoutingIndexManager& manager, const RoutingModel& routing, const Assignment& solution)
{
    compact_solution c_solution;

    const RoutingDimension& time_dimension = routing.GetDimensionOrDie("Time");
    
    for (int vehicle_id = 0; vehicle_id < node.vehicles; ++vehicle_id)
    {
        int64 index = routing.Start(vehicle_id);
        bool not_empty = false;

        //skip depot from solution (routes in the final solution start with the first node served not 0)
        index = solution.Value(routing.NextVar(index));

        //if the route is not ended skipping the depot, initialize a new route. (routes 0 -> 0 are excluded from solution)
        if (!routing.IsEnd(index))
        {
            not_empty = true;
            c_solution.routes.push_back(std::vector<int>());
        }
        int64 route_load{ 0 };

        //add all the customers belonging to the route
        while (!routing.IsEnd(index))
        {
            int node_index = manager.IndexToNode(index).value();
            c_solution.routes[c_solution.number_vehicles].push_back(node.id[node_index]);
            auto time_var = time_dimension.CumulVar(index);
            index = solution.Value(routing.NextVar(index));
        }

        //if route is not (0 -> 0), save information about it
        if (not_empty)
        {
            auto time_var = time_dimension.CumulVar(index);
            c_solution.route_cost.push_back(solution.Min(time_var));
            c_solution.total_cost += c_solution.route_cost[c_solution.number_vehicles];
            c_solution.number_vehicles++;
        }
    }
    
    return c_solution;
}



