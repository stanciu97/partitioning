#pragma once
#include "NodesDistance.h"

/*
* class that implements K-medoid partitioning. 
* 
*/

class KMedoid
{
public:
	/**
	* costructor
	*
	* input:
	* nodes: reference to an istance of NodeDistance, determines the distance used in the algorithm (Euclidean, spatiotemporal)
	*
	*/
	KMedoid(NodesDistance& nodes);

	/**
	* function that creates a partition based on the best seeds found after a certain number of attempts
	*
	* input:
	* groups: total number of clusters in the partition
	* n_iter: number of times the algorithm is repeated
	*
	* output:
	* best found partition after n_iter
	*
	*/
	std::vector<std::vector<int>> medoid_part(int groups);
	std::vector<std::vector<int>> medoid_part(int groups, int n_iter);

private:
	NodesDistance* nodes;
	int iterations = 150;

};
