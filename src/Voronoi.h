#pragma once
#include "NodesDistance.h"
#include "SpatioTemporal.h"
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullVertex.h>
#include<list>
#include<set>

/*
* class that implements a partition based on a Voronoi diagram. 
*
*/

class Voronoi
{
public:

	/**
	* costructor
	*
	* input:
	* node: reference to an existing struct nodes
	* distance: reference to an istance of NodeDistance, determines the distance used in the algorithm (Euclidean, spatiotemporal)
	*
	*/
	Voronoi(nodes& node, NodesDistance& distance, bool use_balanced = true);

	/**
	* function that makes a partition of the customers using a Voronoi diagram.
	*
	* input:
	* n_part: number of seeds used. It is not equal to the number of final groups of the partition
	* n_iter: number of times the algorithm restarts
	*
	* output:
	* n_part groups forming the partition, the solution is the best found after n_iter iterations
	*
	*/
	std::vector<std::vector<int>> voronoi_part(int n_part);
	std::vector<std::vector<int>> voronoi_part(int n_part, int n_iter);
	std::vector<std::vector<int>> voronoi_part_bubble(int n_part);

private:
	nodes* node;
	NodesDistance* distance;
	bool balanced = true;
	int max = 0;
	
	std::vector<orgQhull::QhullVertex> indexed_vertex;

	//object used to compute the voronoi diagram
	orgQhull::Qhull qhull;

	int n_iter = 100;

	//adds (if exists) an element to a cluster. Returns the distance between second last and last inserted element
	double grow_cluster(int& total_size, std::vector<int>& group, std::vector<bool>& inserted, std::list<int>& queue);

	//assign left elements to the best group, considering the distance between the gravity points of the groups and left elements
	void queue_left(std::vector<std::vector<int>>& groups, std::vector<bool>& inserted, std::vector<std::list<int>>& assign_group, std::vector<int>& gravity);

	std::vector<int> find_neighbours(int id);
	
	std::pair<int, double> strongest_partition(std::vector<int>& seed, std::vector<std::vector<int>>& group);
	std::pair<int, double> balanced_partition(std::vector<int>& seed, std::vector<std::vector<int>>& group);

	bool balance(std::vector<int>& seed, std::vector<std::vector<int>>& groups);

	std::vector<int> generate_seed(int n_part);

	template <typename T>  
	void update_candidate(std::vector<bool>& inserted, int customer, T& candidate);

	template <typename T>
	void init_groups(std::vector<bool>& inserted,  std::vector<int>& seed, std::vector<std::vector<int>>& group, std::vector<T>& candidate);

};
